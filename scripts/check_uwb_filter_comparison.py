#!/usr/bin/env python3
"""Offline checks for recorded raw/online-filtered UWB paper figures."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import math

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('flight_log', ROOT/'src/flapping_uav_gcs/scripts/flight_log.py')
LOG=importlib.util.module_from_spec(spec);spec.loader.exec_module(LOG)


class UwbFilterComparisonTests(unittest.TestCase):
    def test_recorded_samples_gaps_identical_references_and_bounds(self):
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib.figure import Figure
        with tempfile.TemporaryDirectory() as temp:
            session=Path(temp);(session/'parameters').mkdir();(session/'config').mkdir()
            tables={
                'trajectory.csv':[{'t_s':str(t),'x_m':str(x),'y_m':str(y)} for t,x,y in
                    [(0,.1,-.2),(.1,.5,.2),(.8,1.2,.7)]],
                'uwb_filtered.csv':[{'t_s':str(t),'x_m':str(x),'y_m':str(y)} for t,x,y in
                    [(0,.05,-.1),(.1,.2,.1),(.8,.8,.4)]],
                'reference.csv':[{'t_s':str(t),'goal_x_m':str(x),'goal_y_m':str(y),
                    'goal_active':'1','trajectory_running':'1'} for t,x,y in
                    [(0,0,0),(.1,.3,.1),(.8,1,.5)]]}
            for name,rows in tables.items():
                with (session/name).open('w',newline='') as stream:
                    writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
            original={name:(session/name).read_bytes() for name in tables}
            captured=[];save=Figure.savefig
            def capture(fig,path,*args,**kwargs):
                if str(path).endswith('.png'):
                    captured.append({'axes':[(ax.get_xlim(),ax.get_ylim()) for ax in fig.axes],
                        'raw_x':list(fig.axes[0].lines[0].get_xdata()),
                        'desired_xy':list(zip(fig.axes[0].lines[1].get_xdata(),fig.axes[0].lines[1].get_ydata()))})
                return save(fig,path,*args,**kwargs)
            with patch.object(Figure,'savefig',new=capture):meta=LOG.uwb_filter_comparison(session,tables)
            self.assertEqual(meta['plots'],['trajectory_uwb_raw','trajectory_uwb_filtered'])
            self.assertEqual(captured[0]['axes'],captured[1]['axes'])
            self.assertEqual([v for v in captured[0]['raw_x'] if math.isfinite(v)],[.1,.5,1.2])
            self.assertEqual([v for v in captured[1]['raw_x'] if math.isfinite(v)],[.05,.2,.8])
            self.assertTrue(math.isnan(captured[0]['raw_x'][2]))
            self.assertTrue(math.isnan(captured[1]['raw_x'][2]))
            # Compare real reference points; matching NaN gap markers are expected.
            references=[[p for p in item['desired_xy'] if all(math.isfinite(v) for v in p)] for item in captured]
            self.assertEqual(references[0],references[1]);self.assertEqual(len(references[0]),3)
            self.assertFalse(meta['offline_filtering']);self.assertFalse(meta['resampled'])
            for name,data in original.items():self.assertEqual((session/name).read_bytes(),data)
            for name in meta['plots']:
                self.assertTrue((session/'report'/(name+'.png')).is_file())
                self.assertTrue((session/'report'/(name+'.pdf')).read_bytes().startswith(b'%PDF'))

    def test_missing_online_recording_does_not_generate_fake_filtered_figure(self):
        with tempfile.TemporaryDirectory() as temp:
            session=Path(temp)
            meta=LOG.uwb_filter_comparison(session,{'trajectory.csv':[{'t_s':'0','x_m':'1','y_m':'2'}]})
            self.assertEqual(meta['plots'],[])
            self.assertIn('uwb_filtered.csv',meta['missing_data'])
            self.assertFalse((session/'report').exists())


if __name__=='__main__':unittest.main(verbosity=2)
