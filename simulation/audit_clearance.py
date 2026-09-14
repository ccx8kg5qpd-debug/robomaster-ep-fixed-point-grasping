"""Conservative convex-hull collision audit of the actual SDF mesh resources.

This is a geometry check, not a replacement for Gazebo contact verification.
"""
from ep_kinematics import *
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
import itertools

def pose(text):
    v=list(map(float,text.split()));t=np.eye(4)
    t[:3,3]=v[:3];t[:3,:3]=Rotation.from_euler('xyz',v[3:]).as_matrix();return t

def vertices(c):
    g=c.find('geometry');mesh=g.find('mesh')
    if mesh is not None:
        doc=E.parse(mesh.findtext('uri').replace('file://',''))
        ns={'d':'http://www.collada.org/2005/11/COLLADASchema'}
        sources=doc.findall('.//d:source',ns)
        a=next(s.find('d:float_array',ns) for s in sources if 'positions' in s.get('id',''))
        v=np.array(list(map(float,a.text.split()))).reshape(-1,3)
        mat=doc.find('.//d:visual_scene/d:node/d:matrix',ns)
        if mat is not None:
            t=np.array(list(map(float,mat.text.split()))).reshape(4,4)
            v=(t@np.c_[v,np.ones(len(v))].T).T[:,:3]
        v*=np.array(list(map(float,mesh.findtext('scale','1 1 1').split())))
    elif g.find('box') is not None:
        s=np.array(list(map(float,g.findtext('box/size').split())))/2
        v=np.array(list(itertools.product([-1,1],repeat=3)))*s
    else:return None
    return (pose(c.findtext('pose','0 0 0 0 0 0'))@np.c_[v,np.ones(len(v))].T).T[:,:3]

def faces(c,v):
    mesh=c.find('geometry/mesh')
    if mesh is None:return ConvexHull(v).simplices
    ns={'d':'http://www.collada.org/2005/11/COLLADASchema'}
    doc=E.parse(mesh.findtext('uri').replace('file://',''));out=[]
    for tri in doc.findall('.//d:triangles',ns):
        inputs=tri.findall('d:input',ns);stride=max(int(i.get('offset')) for i in inputs)+1
        off=int(next(i for i in inputs if i.get('semantic')=='VERTEX').get('offset'))
        idx=np.array(list(map(int,tri.findtext('d:p',namespaces=ns).split()))).reshape(-1,stride)[:,off]
        out.extend(idx.reshape(-1,3))
    if not out:raise ValueError('Unsupported mesh topology')
    return np.array(out)

def edge_hits(v,f,b,bf):
    edges=np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]);o=v[edges[:,0]][:,None,:];d=(v[edges[:,1]]-v[edges[:,0]])[:,None,:]
    t=b[bf];e1=t[:,1]-t[:,0];e2=t[:,2]-t[:,0]
    h=np.cross(d,e2);a=np.sum(e1*h,axis=2);ok=abs(a)>1e-12
    inv=np.divide(1.,a,out=np.zeros_like(a),where=ok);s=o-t[:,0]
    u=inv*np.sum(s*h,axis=2);q=np.cross(s,e1);vv=inv*np.sum(d*q,axis=2);dist=inv*np.sum(e2*q,axis=2)
    return bool(np.any(ok&(u>1e-7)&(vv>1e-7)&(u+vv<1-1e-7)&(dist>1e-7)&(dist<1-1e-7)))

class Audit:
    def __init__(self):
        self.k=Kinematics();self.geometry={};self.faces={}
        for n,e in self.k.nodes.items():
            if e.tag!='link':continue
            for c in e.findall('collision'):
                v=vertices(c)
                if v is not None:
                    key=(n,c.get('name'));self.geometry[key]=v;self.faces[key]=faces(c,v)
    def clouds(self,q):
        f=self.k.frames(q)
        return {(n,c):(f[n]@np.c_[v,np.ones(len(v))].T).T[:,:3] for (n,c),v in self.geometry.items()}
    def check(self,q):
        clouds=self.clouds(q);hits=[];floor=[]
        base=[(n,v) for (link,n),v in clouds.items() if link=='base_link']
        for (link,n),v in clouds.items():
            if not ('gripper' in link or link in ('arm_2_link','endpoint_bracket_link')):continue
            if v[:,2].min()<.0605:floor.append((link,float(v[:,2].min())))
            for bn,b in base:
                if np.any(v.max(0)<b.min(0)) or np.any(b.max(0)<v.min(0)):continue
                eq=np.r_[ConvexHull(v).equations,ConvexHull(b).equations]
                # Largest ball within the hull intersection: >0 means penetration.
                ans=linprog([0,0,0,-1],A_ub=np.c_[eq[:,:3],np.ones(len(eq))],b_ub=-eq[:,3],bounds=[(None,None)]*3+[(0,None)],method='highs')
                if ans.success and ans.x[3]>.0002:
                    vf=self.faces[link,n];bf=self.faces['base_link',bn]
                    if edge_hits(v,vf,b,bf) or edge_hits(b,bf,v,vf):
                        hits.append((link,bn,round(float(ans.x[3])*1000,2)))
        return hits,floor

if __name__=='__main__':
    a=Audit()
    for x in [.232,.24,.245,.25,.255,.26]:
        for z in [.079,.082,.085,.088]:
            try:
                results=[]
                for g in [0,.4,.75]:
                    r,l=a.k.solve(x,z,g);h,f=a.check(joints(r,l,g));results.append((g,h,f))
                print('TARGET',x,z,results,flush=True)
            except ValueError:pass
