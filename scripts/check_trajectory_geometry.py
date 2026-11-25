#!/usr/bin/env python3
"""Offline anchor-to-map trajectory geometry tests; no ROS or devices."""
import importlib.util
import math
from pathlib import Path
import unittest
import yaml
ROOT=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('trajectory_geometry',ROOT/'src/flapping_uav_bringup/launch/trajectory_geometry.py')
G=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(G)
class GeometryTests(unittest.TestCase):
    def setUp(self):
        # Fixed unit-test fixture, independent from a user's new calibration.
        self.scene={'anchor_coordinates_configured':True,'anchor_positions_m':[
            0.,0.,0., .146,5.659,0., 6.543,5.683,0., 6.294,0.,0.]}
        self.uwb={'position_scale':1.,'uwb_y_sign':-1.,'uwb_to_map_yaw_rad':0.,'uwb_to_map_translation_m':[0.,0.,0.]}
    def test_fixed_fixture_line_is_three_metres_and_eight_fits_full_xy_envelope(self):
        g=G.geometry(self.scene,self.uwb,.8,.9,3.)
        self.assertAlmostEqual(2*g['line_half_extents'][0],3.)
        self.assertEqual(g['line_half_extents'][1],0.)
        self.assertAlmostEqual(g['center'][0],3.2715);self.assertAlmostEqual(g['center'][1],-2.8415)
        poly=list(zip(g['polygon'][::2],g['polygon'][1::2]))
        for sx in [-1,1]:
            for sy in [-1,1]:
                self.assertTrue(G.inside_margin((g['center'][0]+sx*g['figure8_half_extents'][0],g['center'][1]+sy*g['figure8_half_extents'][1]),poly,.8))
    def test_world_transform_is_applied_before_path_fitting(self):
        self.uwb['uwb_to_map_yaw_rad']=math.pi/2;self.uwb['uwb_to_map_translation_m']=[10.,20.,0.]
        g=G.geometry(self.scene,self.uwb,.8,.9,3.)
        self.assertAlmostEqual(g['center'][0],12.8415);self.assertAlmostEqual(g['center'][1],23.2715)
    def test_unconfigured_degenerate_or_oversized_paths_rejected(self):
        with self.assertRaises(ValueError):G.geometry(self.scene,self.uwb,.8,.9,100.)
        self.scene['anchor_coordinates_configured']=False
        with self.assertRaises(ValueError):G.geometry(self.scene,self.uwb,.8,.9,3.)
    def test_smaller_profile_reduces_both_paths_without_moving_center(self):
        large=G.geometry(self.scene,self.uwb,.8,.9,3.)
        small=G.geometry(self.scene,self.uwb,.8,.6,2.)
        self.assertEqual(small['center'],large['center'])
        self.assertAlmostEqual(2*small['line_half_extents'][0],2.)
        for k in [0,1]:self.assertLess(small['figure8_half_extents'][k],large['figure8_half_extents'][k])
    def test_current_calibration_and_configured_paths_fit_the_boundary(self):
        scene=yaml.safe_load((ROOT/'src/flapping_uav_bringup/config/scene.yaml').read_text())['scene_visualizer_node']['ros__parameters']
        uwb=yaml.safe_load((ROOT/'src/flapping_uav_bringup/config/uwb.yaml').read_text())['uwb_position_node']['ros__parameters']
        c=yaml.safe_load((ROOT/'src/flapping_uav_bringup/config/trajectory.yaml').read_text())['trajectory']
        g=G.geometry(scene,uwb,c['boundary_margin_m'],c['figure8_fit_scale'],c['line_length_m'],
                     c.get('figure8_axis','x'),c.get('figure8_lobe_aspect_ratio'))
        polygon=list(zip(g['polygon'][::2],g['polygon'][1::2]))
        self.assertAlmostEqual(2*g['line_half_extents'][0],c['line_length_m'])
        for i in range(2001):
            phase=2*math.pi*i/2000
            along_y=g['figure8_axis']=='y'
            eight=(g['center'][0]+g['figure8_half_extents'][0]*math.sin((2 if along_y else 1)*phase),
                   g['center'][1]+g['figure8_half_extents'][1]*math.sin((1 if along_y else 2)*phase))
            line=(g['center'][0]-g['line_half_extents'][0]*math.cos(phase),g['center'][1])
            self.assertTrue(G.inside_margin(eight,polygon,c['boundary_margin_m']))
            self.assertTrue(G.inside_margin(line,polygon,c['boundary_margin_m']))
        self.scene['anchor_coordinates_configured']=True;self.scene['anchor_positions_m']=[0.]*12
        with self.assertRaises(ValueError):G.geometry(self.scene,self.uwb,.8,.9,3.)
    def test_auto_orientation_and_round_lobes_follow_long_side_after_map_rotation(self):
        # Fixture is longer in map X; rotate it 90 degrees to make map Y long.
        for yaw,expected in [(0.,'x'),(math.pi/2,'y')]:
            self.uwb['uwb_to_map_yaw_rad']=yaw
            g=G.geometry(self.scene,self.uwb,.8,.6,2.,'auto',1.)
            self.assertEqual(g['figure8_axis'],expected)
            hx,hy=g['figure8_half_extents']
            self.assertAlmostEqual(hx/(2*hy) if expected=='x' else hy/(2*hx),1.)
        for axis,aspect in [('z',1.),('auto',0.),('x',math.inf)]:
            with self.assertRaises(ValueError):G.geometry(self.scene,self.uwb,.8,.6,2.,axis,aspect)
if __name__=='__main__':unittest.main(verbosity=2)
