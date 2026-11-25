#!/usr/bin/env python3
"""Index recorded sessions and hash publishable logs, without editing samples."""
import argparse
from collections import Counter
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE_DIRS=(
    'experiments','velocity_tests','attitude_tests','analysis',
    'flight_follow_analysis_20261124','flight_response_analysis_20261125',
    'flight_response_analysis_20261125_045132','flight_response_analysis_20261125_051835',
    'uwb_filter_review_20261125','validation')
EXCLUDED_PARTS={'.matplotlib','__pycache__'}
EXCLUDED_NAMES={'.export.lock','.DS_Store'}


def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
    return value.hexdigest()


def read_json(path,default=None):
    if not path.is_file():return {} if default is None else default
    return json.loads(path.read_text(encoding='utf-8-sig'))


def csv_count(path):
    with path.open(encoding='utf-8-sig',newline='') as stream:
        reader=csv.reader(stream);next(reader,None)
        return sum(1 for row in reader if row)


def archived_files(root):
    log=root/'log';result=[]
    for directory in ARCHIVE_DIRS:
        folder=log/directory
        if not folder.is_dir():continue
        for path in folder.rglob('*'):
            relative=path.relative_to(folder)
            if path.is_symlink() or not path.is_file() or any(p in EXCLUDED_PARTS for p in relative.parts):continue
            if path.name in EXCLUDED_NAMES or path.suffix in ('.pyc','.pyo'):continue
            result.append(path)
    for name in ('README.md','COLCON_IGNORE'):
        if (log/name).is_file():result.append(log/name)
    return sorted(result)


def scalar(parameters,key):
    value=parameters.get(key,{})
    if not isinstance(value,dict):return value
    text=value.get('value_text','')
    try:return json.loads(text)
    except (TypeError,ValueError):return text


def index(root):
    directory=root/'log/experiments';directory.mkdir(parents=True,exist_ok=True)
    rows=[];issues=[]
    for path in sorted(directory.glob('*/session.json')):
        m=read_json(path);folder=path.parent;files=m.get('files',{})
        p=read_json(folder/'parameters/position_controller_node.json')
        bridge=read_json(folder/'parameters/mavlink_bridge_node.json')
        mode=scalar(p,'horizontal_control_mode')
        if not mode:
            mode='attitude' if files.get('attitude_desired.csv',{}).get('rows',0)>0 else 'velocity' if files.get('velocity_desired.csv',{}).get('rows',0)>0 else 'unknown'
        before=len(issues)
        for name,meta in files.items():
            source=folder/name
            if not source.is_file():issues.append({'session':folder.name,'file':name,'issue':'missing_file'});continue
            actual=csv_count(source)
            if actual!=meta.get('rows'):issues.append({'session':folder.name,'file':name,'issue':'manifest_row_count_mismatch','manifest_rows':meta.get('rows'),'actual_rows':actual})
        end=m.get('stop_steady_ns',0);start=m.get('start_steady_ns',0)
        duration=round((end-start)/1e9,3) if end>=start and end>0 else ''
        attitude=mode=='attitude'
        row={'session':folder.name,'state':m.get('state','unknown'),'flight_mode':m.get('flight_mode','unknown'),
             'control_mode':mode,'recording_start_trigger':m.get('recording_start_trigger','legacy_heading_alignment'),
             'duration_s':duration,'raw_uwb_rows':files.get('trajectory.csv',{}).get('rows',''),
             'online_filtered_uwb_rows':files.get('uwb_filtered.csv',{}).get('rows',''),
             'reference_rows':files.get('reference.csv',{}).get('rows',''),
             'attitude_rows':files.get('attitude.csv',{}).get('rows',''),
             'flow_actual_rows':files.get('velocity_actual.csv',{}).get('rows',''),
             'pitch_roll_desired_rows':files.get('attitude_desired.csv',{}).get('rows',''),
             'velocity_desired_rows':files.get('velocity_desired.csv',{}).get('rows',''),
             'kp_x':scalar(p,'attitude_kp_x_deg_per_m' if attitude else 'kp_x'),
             'kp_y':scalar(p,'attitude_kp_y_deg_per_m' if attitude else 'kp_y'),
             'ki_x':scalar(p,'attitude_ki_x_deg_per_m_s') if attitude else '',
             'ki_y':scalar(p,'attitude_ki_y_deg_per_m_s') if attitude else '',
             'kd_x':scalar(p,'attitude_kd_x_deg_s_per_m' if attitude else 'kd_x'),
             'kd_y':scalar(p,'attitude_kd_y_deg_s_per_m' if attitude else 'kd_y'),
             'integral_limit_deg':scalar(p,'attitude_integral_limit_deg') if attitude else '',
             'trajectory_period_s':scalar(p,'trajectory_period_s'),
             'optical_flow_fallback_enabled':m.get('flow_assistance_enabled'),
             'velocity_protocol':scalar(bridge,'velocity_protocol'),
             'writer_dropped_rows':m.get('dropped_messages',0),'recording_error':m.get('recording_error',''),
             'csv_integrity_issues':len(issues)-before,
             'report_available':(folder/'report.html').is_file()}
        rows.append(row)
    fields=list(rows[0]) if rows else ['session','state','flight_mode','control_mode']
    with (directory/'catalog.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    text=['# 实机实验原始记录索引','',
          f'本次归档共 **{len(rows)} 份会话**。目录来自真实设备实验记录路径，可能包含地面检查、手动干预、未完成轨迹；不凭目录名判定已起飞或完整自动飞行。',
          '','[catalog.csv](catalog.csv)给出模式、开录触发、样本数、当时PID和记录完整性；参数为空表示没有保存该项，不补成当前默认值。',
          '','- 原始UWB：每目录的trajectory.csv；在线滤波UWB：uwb_filtered.csv（较早日志可能没有）。',
          '- 参考：reference.csv；控制位置：control_position.csv；回传/发送/分项/事件按实际存在的CSV保存。',
          '- config/和parameters/是当时配置/运行快照；解释历史数据时优先于当前源码配置。',
          '- report/保留已有PNG/PDF，report.html可下载后本地打开；GitHub不直接运行HTML。',
          '- 原始文件未重采样、未离线滤波、未按结果删点；NaN/空表、失败和准备阶段按原记录保留。',
          '- log/analysis下的裁剪、速度估计和滤波重放是离线分析，不能替代原始点或当作实际回传。',
          '- source_ros_ns/receive_ros_ns保留采样/接收时间。旧t_s从航向对齐计时，新轨迹日志从TRACKING开录计时，须看manifest。',
          '- 新开录逻辑仍持续到Ctrl+C，FINISHED后保持/降落可能存在；初始alignment元数据的源时间可早于开录。',
          '','| 会话（本地命名时间） | 模式/输出 | 开录 | 时长s | 原始UWB点 | 滤波点 | 记录状态 |',
          '|---|---|---|---:|---:|---:|---|']
    for r in rows:
        trigger='TRACKING' if r['recording_start_trigger']=='trajectory_tracking' else '对齐/旧格式'
        state=r['state']+('；需核对CSV' if r['csv_integrity_issues'] else '')
        text.append(f"| [{r['session']}]({r['session']}/) | {r['flight_mode']} / {r['control_mode']} | {trigger} | {r['duration_s']} | {r['raw_uwb_rows']} | {r['online_filtered_uwb_rows']} | {state} |")
    text += ['','## 完整性说明','',f'按session.json逐表核对存在性及CSV行数，发现 **{len(issues)} 项**需要核对的问题。问题明细在[归档清单](../archive_inventory.json)中；保留原文件，不修补或删除会话。',
             '','归档校验：在工程根目录运行 `python3 scripts/index_experiments.py --verify` 或 `sha256sum --quiet -c log/checksums.sha256`。',
             '','`kp_x/kp_y`在attitude输出时单位°/m，velocity输出时为旧速度环单位；I/D同样按对应模式解释。catalog不把不同协议的数值混作同一种增益。']
    (directory/'README.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
    payload=archived_files(root);size=sum(p.stat().st_size for p in payload);extensions=Counter(p.suffix.lower() or '(none)' for p in payload)
    inventory={'archive_kind':'real_device_sessions_and_explicitly_labelled_derived_analysis',
               'generated_at':datetime.now().astimezone().isoformat(),'session_count':len(rows),
               'payload_files_excluding_inventory_and_checksums':len(payload),
               'payload_bytes_excluding_inventory_and_checksums':size,'file_extensions':dict(extensions),
               'source_samples_modified_by_indexer':False,'session_csv_integrity_issues':issues,
               'included_log_directories':[d for d in ARCHIVE_DIRS if (root/'log'/d).is_dir()],
               'excluded':['build','install','external dependency checkouts','log/experiments_demo','colcon logs','log/ros','Matplotlib caches','old unused FC patch'],
               'notes':'Some hardware sessions may include ground checks or intervention. No ground truth or successful flight claim is inferred. Validation summaries are synthetic software evidence only.'}
    inv=root/'log/archive_inventory.json';inv.write_text(json.dumps(inventory,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    payload.append(inv);payload=sorted(payload)
    lines=[digest(path)+'  '+path.relative_to(root).as_posix() for path in payload]
    (root/'log/checksums.sha256').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps({'sessions':len(rows),'archived_files':len(payload),'archived_bytes':size+inv.stat().st_size,
                      'csv_integrity_issues':len(issues),'raw_data_modified':False},ensure_ascii=False,indent=2))


def verify(root):
    count=0;failed=[]
    for line in (root/'log/checksums.sha256').read_text(encoding='utf-8').splitlines():
        expected,name=line.split('  ',1);path=(root/name).resolve();path.relative_to(root.resolve())
        if not path.is_file() or digest(path)!=expected:failed.append(name)
        count+=1
    print(json.dumps({'verified_files':count,'failed_files':failed,'status':'failed' if failed else 'passed'},ensure_ascii=False,indent=2))
    return 1 if failed else 0


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--verify',action='store_true');parser.add_argument('--root',type=Path,default=ROOT)
    args=parser.parse_args()
    if args.verify:raise SystemExit(verify(args.root))
    index(args.root)


if __name__=='__main__':main()
