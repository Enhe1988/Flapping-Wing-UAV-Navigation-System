#!/usr/bin/env python3
"""Offline exporter regressions. No DDS initialization or physical devices."""
import csv
import fcntl
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('flight_log', ROOT/'src/flapping_uav_gcs/scripts/flight_log.py')
LOG = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LOG)


class ExportTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.session = Path(self.temporary.name)
        (self.session/'config').mkdir()
        (self.session/'parameters').mkdir()
        self.manifest = {'state': 'recorded', 'start_ros_ns': 1000000000,
                         'start_steady_ns': 2000000000, 'stop_steady_ns': 5000000000, 'topics': {}}
        LOG.atomic_json(self.session/'session.json', self.manifest)

    def test_raw_csv_preserves_late_columns_nan_unicode_and_newlines(self):
        import numpy as np
        with LOG.RawCsvSpool(self.session) as spool:
            spool.append('/joy', {'t_s': 0., 'axes[0]': math.nan, 'text': '起飞\n备注', 'channel': np.uint16(1500)})
            spool.append('/joy', {'t_s': 1., 'axes[0]': 1., 'axes[1]': -1., 'text': '恢复'})
            spool.finish()
        with (self.session/'joy.csv').open(encoding='utf-8-sig', newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(rows[0]['axes[1]'], '')
        self.assertTrue(math.isnan(float(rows[0]['axes[0]'])))
        self.assertEqual(rows[0]['text'], '起飞\n备注')
        self.assertEqual(rows[0]['channel'], '1500')
        self.assertEqual(rows[1]['axes[1]'], '-1.0')

    def test_streamed_recovery_duration_and_receive_gap(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_msgs.msg import String
        self.manifest['state'] = 'recording'
        LOG.atomic_json(self.session/'session.json', self.manifest)
        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = 1.
        samples = [('/uwb/position', 1000000000, 2000000000, 0., pose),
                   ('/uwb/position', 3000000000, 4000000000, 2., pose),
                   ('/experiment/mark', 4000000000, 5000000000, 3., String(data='末尾'))]
        with LOG.RawCsvSpool(self.session) as spool, patch.object(LOG, 'records', return_value=iter(samples)):
            manifest, _, _, raw, _, _, _, _ = LOG.read_session(self.session, recover=True, raw_sink=spool)
            spool.finish()
        self.assertEqual(raw, {})
        self.assertEqual(manifest['stop_steady_ns'], 5000000000)
        health = manifest['topic_health']['/uwb/position']
        self.assertEqual(health['count'], 2)
        self.assertAlmostEqual(health['mean_receive_rate_hz'], .5)
        self.assertAlmostEqual(health['max_receive_gap_s'], 2.)
        self.assertIn('Interrupted', manifest['recording_error'])

    def test_export_error_is_persistent_and_next_attempt_can_finish(self):
        original = (self.session/'session.json').read_bytes()
        with patch.object(LOG, '_export_session', side_effect=RuntimeError('drawing failed')):
            with self.assertRaisesRegex(RuntimeError, 'drawing failed'):
                LOG.export_session(self.session)
        failed = LOG.load_json(self.session/'export_status.json')
        self.assertEqual(failed['state'], 'error')
        self.assertIn('drawing failed', failed['error'])
        with patch.object(LOG, '_export_session', return_value={'ok': True}):
            self.assertEqual(LOG.export_session(self.session), {'ok': True})
        self.assertEqual(LOG.load_json(self.session/'export_status.json')['state'], 'finished')
        self.assertEqual((self.session/'session.json').read_bytes(), original)

    def test_concurrent_export_rejected_without_touching_status(self):
        with (self.session/'.export.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(RuntimeError, 'already running'):
                LOG.export_session(self.session)
        self.assertFalse((self.session/'export_status.json').exists())

    def test_active_recording_requires_explicit_recovery(self):
        self.manifest['state'] = 'recording'
        LOG.atomic_json(self.session/'session.json', self.manifest)
        with self.assertRaisesRegex(ValueError, 'still recording'):
            LOG.export_session(self.session)
        self.assertFalse((self.session/'export_status.json').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
