#!/usr/bin/env python3
"""Read-only UDP diagnostic monitor; independent of the ROS communication bridge."""

import argparse
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import selectors
import signal
import socket
import sys
import time


def serializable(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, (bytes, bytearray)):
        return value.hex()
    if isinstance(value, dict):
        return {str(key): serializable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [serializable(item) for item in value]
    return value


def inspect(args):
    if args.pymavlink_path:
        sys.path.insert(0, str(Path(args.pymavlink_path).resolve()))
    try:
        from pymavlink.dialects.v20 import common as mavlink
    except ImportError as error:
        raise RuntimeError('需要调试依赖 pymavlink；可用 --pymavlink-path 指定隔离安装目录') from error

    prefix = Path(args.output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    summary_path = Path(str(prefix) + '-summary.json')
    frames_path = Path(str(prefix) + '-frames.jsonl')
    selector = selectors.DefaultSelector()
    sockets = []
    streams = {}
    stream_limit = 128
    dropped_sources = 0
    stream_packets = Counter()
    bad_units = Counter()
    unknown_units = Counter()
    payload_samples = {}
    packet_lengths = {}
    records = {}
    errors = {}
    packets = Counter()
    started = time.monotonic()
    stopped = False

    def summary(state):
        report = {
            'recorded_at': datetime.now().astimezone().isoformat(),
            'state': state, 'elapsed_s': round(time.monotonic() - started, 3),
            'read_only': True, 'network_sends': 0,
            'scope': 'IPv4 UDP sockets on listed ports, not a capture of all ports',
            'bound_ports': [sock.getsockname()[1] for sock in sockets],
            'bind_errors': errors, 'udp_packets_by_local_port': dict(packets),
            'unparsed_or_bad_units_by_stream': dict(bad_units),
            'unknown_units_by_stream': dict(unknown_units),
            'udp_packet_lengths_by_stream': packet_lengths,
            'udp_payload_samples_by_stream': payload_samples,
            'ignored_datagrams_over_source_limit': dropped_sources,
            'messages': list(records.values()), 'frames_file': str(frames_path),
        }
        temporary = Path(str(summary_path) + '.tmp')
        temporary.write_text(json.dumps(serializable(report), ensure_ascii=False, indent=2) + '\n')
        temporary.replace(summary_path)
        total = sum(record['count'] for record in records.values())
        print('elapsed={:.1f}s UDP={} MAVLink={} ports={}'.format(
            time.monotonic() - started, sum(packets.values()), total, dict(packets)), flush=True)

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    old_handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for port in sorted(set(args.ports)):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                # Do not reuse or steal an occupied application port.
                sock.bind(('0.0.0.0', port))
            except OSError as error:
                errors[str(port)] = str(error)
                sock.close()
                continue
            sock.setblocking(False)
            sockets.append(sock)
            selector.register(sock, selectors.EVENT_READ)
        summary('listening')
        if not sockets:
            raise RuntimeError('没有可用监听端口；查看 bind_errors')
        next_report = time.monotonic() + 10
        with frames_path.open('w', buffering=1) as output:
            while not stopped and time.monotonic() - started < args.duration:
                for key, _ in selector.select(timeout=0.25):
                    sock = key.fileobj
                    data, peer = sock.recvfrom(65535)
                    local_port = sock.getsockname()[1]
                    packets[local_port] += 1
                    source = '{} <- {}:{}'.format(local_port, *peer)
                    if source not in streams:
                        if len(streams) >= stream_limit:
                            dropped_sources += 1
                            continue
                        streams[source] = mavlink.MAVLink(None)
                        streams[source].robust_parsing = True
                    stream_packets[source] += 1
                    lengths = packet_lengths.setdefault(source, Counter())
                    lengths[len(data)] += 1
                    samples = payload_samples.setdefault(source, [])
                    if len(samples) < 6:
                        samples.append({'bytes': len(data), 'hex_prefix': data[:128].hex()})
                    messages = streams[source].parse_buffer(data) or []
                    for message in messages:
                        # Unknown dialect placeholders have id -2 and names such as
                        # UNKNOWN_57. They are not CRC-validated known messages.
                        if message.get_msgId() not in mavlink.mavlink_map:
                            bad_units[source] += 1
                            if message.get_type().startswith('UNKNOWN'):
                                unknown_units[source] += 1
                            continue
                        now = time.monotonic()
                        identity = '{} sys={} comp={} id={}'.format(
                            source, message.get_srcSystem(), message.get_srcComponent(),
                            message.get_msgId())
                        event = {
                            'received_at': datetime.now().astimezone().isoformat(),
                            'elapsed_s': round(now - started, 6), 'local_port': local_port,
                            'source_ip': peer[0], 'source_port': peer[1],
                            'system_id': message.get_srcSystem(),
                            'component_id': message.get_srcComponent(),
                            'message_id': message.get_msgId(), 'name': message.get_type(),
                            'sequence': message.get_seq(),
                            'wire_version': 2 if message.get_msgbuf()[0] == 0xfd else 1,
                            'fields': message.to_dict(),
                        }
                        output.write(json.dumps(serializable(event), ensure_ascii=False) + '\n')
                        if identity not in records:
                            records[identity] = {
                                'source': source, 'local_port': local_port,
                                'system_id': event['system_id'],
                                'component_id': event['component_id'],
                                'message_id': event['message_id'], 'name': event['name'],
                                'count': 0, 'first_elapsed_s': event['elapsed_s'],
                            }
                            print(json.dumps(serializable(event), ensure_ascii=False), flush=True)
                        record = records[identity]
                        record['count'] += 1
                        record['last_elapsed_s'] = event['elapsed_s']
                        span = record['last_elapsed_s'] - record['first_elapsed_s']
                        record['observed_hz'] = (record['count'] - 1) / span if span > 0 else None
                        record['last_message'] = event
                if time.monotonic() >= next_report:
                    summary('listening')
                    next_report = time.monotonic() + 10
    finally:
        summary('stopped')
        for sock in sockets:
            selector.unregister(sock)
            sock.close()
        selector.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description='只接收的 MAVLink UDP 端口/消息监视器')
    parser.add_argument('--ports', nargs='+', type=int,
                        default=[9000])
    parser.add_argument('--duration', type=float, default=300)
    parser.add_argument('--pymavlink-path', help='隔离安装的诊断依赖目录')
    parser.add_argument('--output-prefix', default='log/mavlink-inspection')
    args = parser.parse_args()
    if any(port < 1 or port > 65535 for port in args.ports):
        parser.error('端口必须在 1..65535')
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration 必须是正数')
    try:
        inspect(args)
    except (OSError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
