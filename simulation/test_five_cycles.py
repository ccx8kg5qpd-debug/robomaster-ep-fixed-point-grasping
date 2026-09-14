"""Offline numerical/structural checks. Runtime evidence is verified separately."""
import json
import unittest
import xml.etree.ElementTree as E
import numpy as np
from ep_kinematics import ROOT,Kinematics,joints

class FiveCycleTests(unittest.TestCase):
    def test_distinct_home(self):
        cfg=json.loads((ROOT/'experiment.json').read_text())
        self.assertEqual(cfg['cycles'],5)
        self.assertGreater(np.linalg.norm(np.array(cfg['home_tip_xz_m'])-cfg['hover_tip_xz_m']),.04)
    def test_targets_and_interpolated_joints_within_model_limits(self):
        k=Kinematics();cfg=json.loads((ROOT/'experiment.json').read_text())
        model=E.parse(ROOT/'ep_feedback.sdf').getroot().find('model')
        limits={j.get('name'):(float(j.findtext('axis/limit/lower','-inf')),float(j.findtext('axis/limit/upper','inf'))) for j in model.findall('joint')}
        goals=[]
        for x,z in (cfg['home_tip_xz_m'],cfg['hover_tip_xz_m'],cfg['grasp_tip_xz_m']):
            for g in (.6,.72,1.):
                r,l=k.solve(x,z,g);q=joints(r,l,g);goals.append(q)
                self.assertLess(np.linalg.norm(k.center(q)[[0,2]]-[x,z]),.002)
                for n,v in q.items():
                    if n in limits:self.assertTrue(limits[n][0]-1e-8<=v<=limits[n][1]+1e-8,(n,v,limits[n]))
        # Joint interpolation stays inside convex scalar joint-limit intervals.
        for a,b in zip(goals,goals[1:]):
            for u in np.linspace(0,1,11):
                for n in a:
                    if n in limits:self.assertTrue(limits[n][0]-1e-8<=(1-u)*a[n]+u*b[n]<=limits[n][1]+1e-8)
    def test_unreachable_rejected(self):
        with self.assertRaises(ValueError):Kinematics().solve(10.,10.,.6)
    def test_refill_is_separate_from_grasp_control(self):
        code=(ROOT/'ep_transfer_control.py').read_text()
        self.assertNotIn('subprocess',code)
        refill=(ROOT/'scene_refill.py').read_text()
        self.assertIn("self.phase!='REFILL_WAIT'",refill)

if __name__=='__main__':unittest.main()
