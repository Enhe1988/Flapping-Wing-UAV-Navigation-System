#!/usr/bin/env python3
"""Offline synthetic regressions for same-flight UWB/flow figures. No hardware."""
import csv
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


def read_rows(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def write_rows(path, rows):
    with path.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


class ComparisonTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='synthetic-localization-')
        self.addCleanup(self.temporary.cleanup)
        self.session = Path(self.temporary.name)
        (self.session/'config').mkdir()
        (self.session/'config/fusion.yaml').write_text(
            'fusion_odometry_node:\n  ros__parameters:\n    uwb_timeout_ms: 150\n')
        raw = []; sources = []; reference = []
        for index in range(151):
            t = index*.04
            x = 2 + .5*math.sin(t); y = -2 + .3*math.sin(2*t)
            prefix = {'t_s': format(t, '.17g'), 'receive_ros_ns': str(1000000000+index*40000000+123456),
                      'source_ros_ns': str(1000000000+index*40000000)}
            if not 60 <= index < 90:
                raw.append(dict(prefix, x_m='nan' if index == 132 else format(x+(2 if 30 <= index < 60 else 0), '.17g'),
                                y_m=format(y-(1 if 30 <= index < 60 else 0), '.17g'), z_uwb_raw_m='42', frame_id='map'))
            valid = not 120 <= index < 126
            source = 'INVALID' if not valid else 'FLOW_ODOMETRY' if 30 <= index < 90 else 'UWB_BLEND' if index < 101 and index >= 90 else 'UWB'
            sources.append(dict(prefix, selected_x_m=format(x if valid else 999, '.17g'),
                selected_y_m=format(y if valid else 999, '.17g'), valid=str(int(valid)), source=source,
                uwb_healthy=str(int(not 30 <= index < 90)), flow_valid=str(int(valid)),
                raw_uwb_age_ms='1000' if 60 <= index < 90 else '0', reliable_uwb_age_ms='123.45678901234567',
                flow_age_ms='nan' if not valid else '0', innovation_m='2.2360679774997898' if 30 <= index < 60 else '0',
                uwb_syncs=str(index//5), odom_x_m=format(x, '.17g'), odom_y_m=format(y, '.17g'),
                raw_uwb_x_m=format(x+(2 if 30 <= index < 60 else 0), '.17g'), raw_uwb_y_m=format(y, '.17g')))
            reference.append(dict(prefix, goal_x_m=format(x, '.17g'), goal_y_m=format(y, '.17g'),
                                  goal_active='1', trajectory_running='1', trajectory_phase='TRACKING'))
        self.tables = {'trajectory.csv': raw, 'position_sources.csv': sources, 'reference.csv': reference}
        for name, rows in self.tables.items(): write_rows(self.session/name, rows)
        self.manifest = {'format': 'paper_csv_v1', 'label': 'synthetic_same_flight', 'state': 'recorded',
                         'flight_mode': 'figure8', 'flow_assistance_enabled': True, 'start_steady_ns': 1,
                         'files': {name: {'rows': len(rows)} for name, rows in self.tables.items()}}
        LOG.atomic_json(self.session/'session.json', self.manifest)
        self.originals = {name: (self.session/name).read_bytes() for name in self.tables}

    def export(self):
        return LOG.localization_comparison(self.session, self.manifest, self.tables)

    def test_two_figures_preserve_raw_samples_precision_flags_and_shared_axes(self):
        meta = self.export(); out = self.session/'localization_comparison'
        self.assertEqual(len(list(out.glob('*.png'))), 2)
        self.assertEqual(len(list(out.glob('*.pdf'))), 2)
        for path in out.glob('*.pdf'): self.assertTrue(path.read_bytes().startswith(b'%PDF'))
        self.assertEqual((out/'uwb_raw_points.csv').read_bytes(), self.originals['trajectory.csv'])
        self.assertEqual((out/'source_status.csv').read_bytes(), self.originals['position_sources.csv'])
        self.assertEqual((out/'reference_points.csv').read_bytes(), self.originals['reference.csv'])
        exported = read_rows(out/'flow_compensated_points.csv')
        self.assertEqual(len(exported), len(self.tables['position_sources.csv']))
        for original, actual in zip(self.tables['position_sources.csv'], exported):
            self.assertEqual(actual['x_m'], original['selected_x_m'])
            self.assertEqual(actual['y_m'], original['selected_y_m'])
            for field in ['t_s', 'source_ros_ns', 'receive_ros_ns', 'valid', 'source', 'flow_age_ms', 'innovation_m']:
                self.assertEqual(actual[field], original[field])
        self.assertEqual(exported[120]['valid'], '0')
        self.assertEqual(exported[120]['x_m'], '999')
        self.assertTrue(math.isnan(float(read_rows(out/'uwb_raw_points.csv')[-19]['x_m'])))
        self.assertLess(meta['xy_limits_m'][0][1], 10)  # Invalid 999 must not set plotting bounds.
        self.assertTrue(meta['flow_fallback_observed'])
        self.assertTrue(meta['synthetic_validation'])
        self.assertEqual(meta['comparison_type'], 'same_experiment_position_estimates')
        self.assertFalse(meta['offline_filtering']); self.assertFalse(meta['resampled'])
        for name, data in self.originals.items(): self.assertEqual((self.session/name).read_bytes(), data)

    def test_gaps_are_broken_and_both_figures_use_identical_limits(self):
        from matplotlib.axes import Axes
        from matplotlib.figure import Figure
        original_plot = Axes.plot; original_save = Figure.savefig
        raw_times = []; limits = []
        def plot(axis, *args, **kwargs):
            if kwargs.get('label') == 'Recorded X' and not raw_times:
                raw_times.append(list(args[0]))
            return original_plot(axis, *args, **kwargs)
        def save(figure, path, *args, **kwargs):
            if str(path).endswith('.png'):
                limits.append([(ax.get_xlim(), ax.get_ylim()) for ax in figure.axes])
            return original_save(figure, path, *args, **kwargs)
        with patch.object(Axes, 'plot', new=plot), patch.object(Figure, 'savefig', new=save):
            meta = self.export()
        resumed = next(i for i, t in enumerate(raw_times[0]) if math.isfinite(t) and abs(t-3.6) < 1e-9)
        self.assertTrue(math.isnan(raw_times[0][resumed-1]))
        self.assertEqual(limits[0], limits[1])
        self.assertEqual(meta['uwb_unhealthy_intervals_s'], [[1.2, 3.6]])
        self.assertEqual(meta['valid_source_counts']['FLOW_ODOMETRY'], 60)

    def test_no_fallback_does_not_claim_compensation_occurred(self):
        for row in self.tables['position_sources.csv']: row.update(source='UWB', uwb_healthy='1')
        self.manifest['flow_assistance_enabled'] = False
        meta = self.export()
        self.assertFalse(meta['flow_fallback_observed'])
        self.assertFalse(meta['assistance_used_by_controller'])
        self.assertEqual(meta['flow_fallback_intervals_s'], [])

    def test_missing_estimator_is_not_replaced_with_controller_or_raw_uwb(self):
        self.tables['position_sources.csv'] = []
        self.tables['control_position.csv'] = [{'t_s': '0', 'x_m': '3', 'y_m': '4', 'valid': '1'}]
        (self.session/'position_sources.csv').unlink()
        meta = self.export()
        self.assertEqual(meta['compensated_point_rows'], 0)
        self.assertEqual(read_rows(self.session/'localization_comparison/flow_compensated_points.csv'), [])
        self.assertFalse(meta['flow_fallback_observed'])

    def test_normal_paper_export_automatically_links_comparison(self):
        summary = LOG.export_session(self.session)
        self.assertEqual(len(list((self.session/'report').glob('*.png'))), 3)
        self.assertEqual(len(list((self.session/'localization_comparison').glob('*.png'))), 2)
        self.assertTrue(summary['localization_comparison']['flow_fallback_observed'])
        page = (self.session/'report.html').read_text()
        self.assertIn('localization_comparison/uwb_raw_trajectory.png', page)
        self.assertIn('localization_comparison/flow_compensated_points.csv', page)
        self.assertEqual(LOG.load_json(self.session/'export_status.json')['state'], 'finished')
        for name, data in self.originals.items(): self.assertEqual((self.session/name).read_bytes(), data)


if __name__ == '__main__':
    unittest.main(verbosity=2)
