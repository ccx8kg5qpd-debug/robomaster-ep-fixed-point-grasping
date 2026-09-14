"""Kinematics from the community robomaster_ros mesh/joint model, SI units."""
import json
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

ROOT = Path(__file__).parent
TABLE = np.array(json.loads((ROOT / 'ep_gripper_table.json').read_text()))
GRIP_NAMES = ['gripper_m_joint'] + [f'{s}_gripper_joint_{i}' for s in ('left','right') for i in (1,2,4,5,6,7)]

def joints(r, l, g, yaw=0.):
    q = dict(arm_1_joint=r, arm_2_joint=l-r, endpoint_bracket_joint=-l,
             rod_joint=l, rod_1_joint=l, rod_2_joint=r, rod_3_joint=r-l,
             triangle_joint=-r, calibration_yaw=yaw)
    for i, n in enumerate(GRIP_NAMES):
        q[n] = float(np.interp(g, np.linspace(0,1,len(TABLE)), TABLE[:,i]))
    return q

class Kinematics:
    def __init__(self):
        m = E.parse(ROOT/'ep_feedback.sdf').getroot().find('model')
        self.nodes = {e.get('name'):e for e in m if e.tag in ('link','joint','frame')}
    def frames(self, q):
        cache = {'__model__':np.eye(4), 'world':np.eye(4)}
        cache['__model__'][2,3] = .06
        def get(n):
            if n in cache: return cache[n]
            e = self.nodes[n]; p = e.find('pose')
            ref = p.get('relative_to') if p is not None else None
            if not ref: ref = e.get('attached_to','__model__')
            values = list(map(float,p.text.split())) if p is not None and p.text else [0]*6
            t = np.eye(4); t[:3,:3] = Rotation.from_euler('xyz',values[3:]).as_matrix(); t[:3,3]=values[:3]
            if e.tag == 'joint' and e.get('type') != 'fixed':
                axis = np.array(list(map(float,e.findtext('axis/xyz').split())))
                d = np.eye(4)
                if e.get('type') == 'prismatic': d[:3,3] = axis*q.get(n,0.)
                else: d[:3,:3]=Rotation.from_rotvec(axis*q.get(n,0.)).as_matrix()
                t = t@d
            cache[n] = get(ref)@t
            return cache[n]
        for n in self.nodes: get(n)
        # base_link has model-relative pose, so yaw is applied to all model frames.
        rot = np.eye(4); rot[:3,:3]=Rotation.from_euler('z',q.get('calibration_yaw',0)).as_matrix()
        return {n:rot@t for n,t in cache.items()}
    def tips(self,q):
        f = self.frames(q)
        # Link 7 is a rear linkage, not a fingertip. Grasp at the forward
        # end of link 4, whose collision mesh extends to x=0.04474 m.
        return [ (f[s+'_gripper_link_4'] @ np.array([.043,.003 if s=='left' else -.003,0,1]))[:3] for s in ('left','right') ]
    def center(self,q): return np.mean(self.tips(q),axis=0)
    def solve(self,x,z,g=0.,seed=(.7,-.2)):
        def err(v):
            r,l=v
            return (self.center(joints(r,l,g))[[0,2]]-[x,z])
        ans=least_squares(err,seed,bounds=([-.274,-.79936],[1.384,1.73137]),xtol=1e-12,ftol=1e-12,gtol=1e-12)
        r,l=ans.x
        if np.linalg.norm(err(ans.x))>.002 or not -.34732 <= r-l <= 1.21475:
            raise ValueError(f'Unreachable target {(x,z)}: angles {ans.x}, residual {err(ans.x)}')
        return r,l

if __name__=='__main__':
    k=Kinematics()
    for g in [0,.3,.6,1]:
        q=joints(.4,0,g); a,b=k.tips(q)
        print('closure',g,'center',k.center(q),'gap',np.linalg.norm(a-b))
    for z in [.12,.15,.20,.24]:
        try: print('target',.24,z,k.solve(.24,z))
        except ValueError as e: print(e)
