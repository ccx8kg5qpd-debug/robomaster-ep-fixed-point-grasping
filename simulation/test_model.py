"""Structural checks; runtime grasp and wheel traction still require Gazebo."""
import json
import unittest
from pathlib import Path
import xml.etree.ElementTree as E
ROOT=Path(__file__).parent
class ModelTests(unittest.TestCase):
    def test_shared_table(self):
        w=E.parse(ROOT/'ep_feedback.world.sdf').getroot().find('world')
        self.assertIsNone(w.find("model[@name='pad_a']"))
        self.assertIsNone(w.find("model[@name='pad_b']"))
        table=w.find("model[@name='table']")
        z=float(table.findtext('pose').split()[2])
        thick=float(table.findtext('link/collision/geometry/box/size').split()[2])
        self.assertAlmostEqual(z+thick/2,.06)
        for name in ('mark_A','mark_B'):
            self.assertFalse(w.find(f"model[@name='{name}']").findall('.//collision'))
    def test_mobile_model(self):
        m=E.parse(ROOT/'ep_feedback.sdf').getroot().find('model')
        self.assertEqual(m.findtext('static'),'false')
        self.assertFalse(any(j.findtext('parent')=='world' for j in m.findall('joint')))
        for s in ['front_left','front_right','rear_left','rear_right']:
            self.assertEqual(m.find(f"joint[@name='{s}_wheel_joint']").get('type'),'revolute')
        names=json.loads((ROOT/'ep_joint_names.json').read_text())
        self.assertNotIn('calibration_yaw',names)
        self.assertTrue(any('MecanumDrive' in p.get('name','') for p in m.findall('plugin')))
    def test_cube_inertia(self):
        w=E.parse(ROOT/'ep_feedback.world.sdf').getroot().find('world')
        for tag in ('collision','visual'):
            size=w.findtext(f"model[@name='pick_box']/link/{tag}/geometry/box/size")
            self.assertEqual(list(map(float,size.split())),[.025,.025,.025])
        i=w.find("model[@name='pick_box']/link/inertial")
        self.assertAlmostEqual(float(i.findtext('inertia/ixx')), .05*.025**2/6)
    def test_no_object_pose_commands(self):
        code=(ROOT/'ep_transfer_control.py').read_text()
        self.assertNotIn('set_pose',code.replace('no set_pose',''))
        self.assertNotIn('subprocess',code)
if __name__=='__main__':unittest.main()
