"""Contact grasp and wheel-driven A-to-B transfer; never sets model poses."""
import json, math, time, csv
from pathlib import Path
from datetime import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from ros_gz_interfaces.msg import Contacts
from ep_kinematics import Kinematics, joints, ROOT, TABLE

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def angle(x):return math.atan2(math.sin(x),math.cos(x))

class Transfer(Node):
    def __init__(self):
        super().__init__('ep_transfer_control')
        self.k=Kinematics();self.q={};self.poses={};self.qtime=0.;self.ptime=0.
        self.contact_time=0.;self.paired=False;self.support=False
        self.active=False;self.phase='IDLE';self.target=None;self.grip=None
        self.cfg=json.loads((ROOT/'experiment.json').read_text())
        self.cycle=0;self.results=[];self.attempted=False;self.refill_future=None
        self.logdir=ROOT/'logs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.logdir.mkdir(parents=True)
        self.events=(self.logdir/'events.jsonl').open('w',buffering=1)
        self.tracefile=(self.logdir/'trajectory.csv').open('w',newline='',buffering=1)
        self.trace=csv.writer(self.tracefile)
        self.trace.writerow(['monotonic_seconds','cycle','phase','base_x','base_y','base_yaw','box_x','box_y','box_z','measured_joints_json','target_joints_json'])
        self.last_trace=0
        self.refill=self.create_client(Trigger,'/pick_place/refill_at_home')
        self.velocity=self.create_publisher(Twist,'/ep/cmd_vel',10)
        self.state=self.create_publisher(String,'/pick_place/state',10)
        self.pubs={n:self.create_publisher(Float64,'/ep/joint/'+n,10) for n in json.loads((ROOT/'ep_joint_names.json').read_text())}
        self.create_subscription(JointState,'/ep/joint_states',self.feedback,10)
        self.create_subscription(TFMessage,'/world/pick_place_world/pose/info',self.pose,10)
        self.create_subscription(Contacts,'/world/pick_place_world/model/pick_box/link/box_link/sensor/box_contacts/contact',self.contacts,10)
        self.create_service(Trigger,'/pick_place/run',self.run)
        self.create_service(Trigger,'/pick_place/stop',self.stop)
        self.create_service(Trigger,'/pick_place/resume_transfer',self.resume)
        self.create_timer(.04,self.tick)
        self.get_logger().info('READY: mobile wheel-drive trial; no set_pose, no grasp attachment')
    def event(self,name,**data):
        record=dict(event=name,cycle=self.cycle,monotonic_seconds=time.monotonic(),**data)
        self.events.write(json.dumps(record,ensure_ascii=False)+'\n')
        self.get_logger().info(json.dumps(record,ensure_ascii=False))
    def save_results(self,status):
        (self.logdir/'results.json').write_text(json.dumps(dict(status=status,requested_cycles=5,completed_cycles=len(self.results),cycles=self.results,configuration=self.cfg),ensure_ascii=False,indent=2))
    def feedback(self,m):
        self.q=dict(zip(m.name,m.position));self.qtime=time.monotonic()
        if self.target is None:self.target={n:self.q.get(n,0.) for n in self.pubs}
    def pose(self,m):
        for t in m.transforms:self.poses[t.child_frame_id]=t.transform
        self.ptime=time.monotonic()
    def contacts(self,m):
        names={c.collision1.name for c in m.contacts}|{c.collision2.name for c in m.contacts}
        self.paired=all(any('robomaster_ep::'+s+'_gripper' in n for n in names) for s in ('left','right'))
        self.support=any('table::' in n for n in names)
        self.contact_time=time.monotonic()
    def point(self,name):
        p=self.poses[name].translation;return np.array([p.x,p.y,p.z])
    def run(self,req,res):
        if self.active or not all(n in self.poses for n in ['robomaster_ep','pick_box']):
            res.success=False;res.message='Busy or model pose feedback missing';return res
        if time.monotonic()-min(self.ptime,self.qtime)>2:
            res.success=False;res.message='Stale feedback';return res
        if np.linalg.norm(self.point('pick_box')[:2]-self.cfg['a_world_m'][:2])>.015:
            res.success=False;res.message='Cube is not at A; reset experiment first';return res
        self.base_origin=self.point('robomaster_ep').copy();self.base_yaw=yaw(self.poses['robomaster_ep'].rotation)
        try:
            for x,z in [self.cfg['home_tip_xz_m'],self.cfg['hover_tip_xz_m'],self.cfg['grasp_tip_xz_m']]:
                for g in (.6,1.):self.k.solve(x,z,g)
        except ValueError as e:
            self.event('PREFLIGHT_REJECTED',reason=str(e));self.save_results('PREFLIGHT_REJECTED')
            res.success=False;res.message=str(e);return res
        self.b=np.array(self.cfg['b_world_m']);self.grip=None;self.active=True
        self.cycle=1;self.results=[];self.turn=0.;self.cycle_started=time.monotonic()
        self.run_started=time.monotonic()
        self.enter('INITIAL_HOME',*self.cfg['home_tip_xz_m'],.6,4)
        self.event('RUN_STARTED');self.save_results('RUNNING')
        res.success=True;res.message='Five fixed A-to-B cycles, each with measured home verification';return res
    def stop(self,req,res):
        self.fail('USER_STOP');res.success=True;return res
    def resume(self,req,res):
        res.success=False;res.message='V2 requires a fresh complete timed run';return res
    def fail(self,why):
        self.active=False;self.velocity.publish(Twist())
        self.target={n:self.q.get(n,0.) for n in self.pubs}
        self.state.publish(String(data='FAILED_'+why));self.get_logger().error(why)
        self.event('SAFE_STOP',reason=why);self.save_results('FAILED_'+why)
    def enter(self,phase,x,z,g,duration):
        self.phase=phase;self.started=time.monotonic();self.duration=duration;self.hold_since=None
        self.start={n:self.q.get(n,0.) for n in self.pubs}
        if self.grip is not None and phase not in ('OPEN','RETREAT','RETURN_HEADING','RETURN_HOME','REFILL_WAIT'):
            g=self.grip
        try:r,l=self.k.solve(x,z,g)
        except ValueError as e:self.fail(str(e));return
        q=joints(r,l,g);self.target={n:q[n] for n in self.pubs}
        self.state.publish(String(data=phase));self.get_logger().info(f'{phase}: x={x:.4f} z={z:.4f} closure={g:.4f}')
        self.event('PHASE_ENTER',phase=phase,target_tip_xz_m=[x,z],closure=g)
    def base_control(self):
        p=self.point('robomaster_ep');a=yaw(self.poses['robomaster_ep'].rotation)
        desired=self.base_yaw+(self.turn if self.phase in ('TURN','LOWER','OPEN','RETREAT','VERIFY') else 0.)
        delta=self.base_origin[:2]-p[:2];c,s=math.cos(a),math.sin(a)
        cmd=Twist();cmd.linear.x=float(np.clip(1.0*(c*delta[0]+s*delta[1]),-.025,.025))
        cmd.linear.y=float(np.clip(1.0*(-s*delta[0]+c*delta[1]),-.025,.025))
        angular_error=angle(desired-a)
        cmd.angular.z=(float(np.clip(1.5*angular_error+math.copysign(.02,angular_error),-.22,.22)) if abs(angular_error)>.012 else 0.)
        self.velocity.publish(cmd)
        return abs(angular_error)<.015 and np.linalg.norm(delta)<.006
    def tick(self):
        if self.target is None:return
        now=time.monotonic()
        if self.active and now-min(self.qtime,self.ptime)>2:self.fail('STALE_FEEDBACK')
        if not self.attempted and self.cfg['autostart'] and all(n in self.poses for n in ['robomaster_ep','pick_box']) and now-min(self.qtime,self.ptime)<1 and self.q:
            self.attempted=True
            response=self.run(Trigger.Request(),Trigger.Response())
            if not response.success:self.event('START_REJECTED',reason=response.message)
        if self.active and now-self.run_started>self.cfg['run_timeout_seconds']:self.fail('RUN_TIME_LIMIT')
        if now-self.last_trace>.1 and all(n in self.poses for n in ('robomaster_ep','pick_box')):
            self.last_trace=now
            self.trace.writerow([now,self.cycle,self.phase,*self.point('robomaster_ep')[:2],yaw(self.poses['robomaster_ep'].rotation),*self.point('pick_box'),json.dumps(self.q),json.dumps(self.target)])
        if self.active:
            base_ok=self.base_control();elapsed=now-self.started
            u=float(np.clip(elapsed/self.duration,0,1));u=u*u*u*(10+u*(-15+6*u))
            command={n:self.start[n]+(v-self.start[n])*u for n,v in self.target.items()}
        else:
            command=self.target;self.velocity.publish(Twist())
        for n,v in command.items():self.pubs[n].publish(Float64(data=float(v)))
        if not self.active:return
        if self.phase=='REFILL_WAIT':
            if self.refill_future is not None and self.refill_future.done():
                try:ok=self.refill_future.result().success
                except Exception:ok=False
                if not ok:self.fail('REFILL_SERVICE_FAILED');return
                self.refill_future=None
                self.event('SCENE_REFILL_ACKNOWLEDGED')
            ready=self.refill_future is None and np.linalg.norm(self.point('pick_box')-self.cfg['a_world_m'])<.008 and self.support and now-self.contact_time<.3
            if ready and elapsed>2:
                self.cycle+=1;self.cycle_started=now;self.grip=None
                self.enter('HOVER',*self.cfg['hover_tip_xz_m'],.6,4)
            elif elapsed>20:self.fail('REFILL_TIMEOUT')
            return
        paired=self.paired and now-self.contact_time<.3
        if self.phase=='CLOSE' and paired:
            g=float(np.interp(-self.q['gripper_m_joint'],-TABLE[:,0],np.linspace(0,1,len(TABLE))))
            self.grip=min(1.,g+.04);self.enter('HOLD',.27,.088,self.grip,3);return
        arm_error=max(abs(self.q.get(n,999)-v) for n,v in self.target.items() if 'gripper' not in n)
        box=self.point('pick_box')
        if self.phase=='TURN' and box[2]<.125:self.fail('OBJECT_DROPPED');return
        if self.phase=='LIFT' and elapsed>self.duration+2 and box[2]<.09:
            self.fail('GRASP_LOST_DURING_LIFT');return
        reached=arm_error<.012 and base_ok and elapsed>self.duration
        if self.phase=='CLOSE':reached=False
        if self.phase=='HOLD':reached=reached and paired
        if self.phase=='LIFT':reached=reached and paired and box[2]>.135
        if self.phase=='LOWER':reached=reached and np.linalg.norm(box[:2]-self.b[:2])<.025 and box[2]<.089
        if elapsed>self.duration+25:self.fail('TIMEOUT_'+self.phase);return
        if not reached:self.hold_since=None;return
        if self.hold_since is None:self.hold_since=now
        if now-self.hold_since<.4:return
        self.get_logger().info(f'{self.phase} verified: cube={box.tolist()} arm_error={arm_error:.5f} wall_seconds={now-self.run_started:.3f}')
        if self.phase=='INITIAL_HOME':
            self.event('INITIAL_HOME_VERIFIED')
            self.enter('HOVER',*self.cfg['hover_tip_xz_m'],.6,4)
        elif self.phase=='HOVER':self.enter('DESCEND',.27,.088,.6,5)
        elif self.phase=='DESCEND':self.enter('CLOSE',.27,.088,1.,6)
        elif self.phase=='HOLD':self.enter('LIFT',.27,.16,self.grip,5)
        elif self.phase=='LIFT':
            self.turn=angle(math.atan2(self.b[1]-self.base_origin[1],self.b[0]-self.base_origin[0])-math.atan2(box[1]-self.base_origin[1],box[0]-self.base_origin[0]))
            self.enter('TURN',.27,.16,self.grip,4)
        elif self.phase=='TURN':
            # Preserve measured cube-to-pad height offset; descend to 2 mm over B.
            self.drop_z=max(.088,.16+(.0745-box[2]));self.enter('LOWER',.27,self.drop_z,self.grip,4)
        elif self.phase=='LOWER':self.enter('OPEN',.27,self.drop_z,.6,3)
        elif self.phase=='OPEN':self.enter('RETREAT',.27,.16,.6,3)
        elif self.phase=='RETREAT':
            if np.linalg.norm(box[:2]-self.b[:2])<.023 and abs(box[2]-.0725)<.008 and self.support and now-self.contact_time<.3:
                self.placement=box.tolist();self.event('B_PLACEMENT_VERIFIED',box_world_m=self.placement)
                self.grip=None
                self.enter('RETURN_HEADING',*self.cfg['hover_tip_xz_m'],.6,4)
            else:self.fail('B_PLACEMENT_NOT_CONFIRMED')
        elif self.phase=='RETURN_HEADING':
            self.enter('RETURN_HOME',*self.cfg['home_tip_xz_m'],.6,4)
        elif self.phase=='RETURN_HOME':
            gripper_error=max(abs(self.q.get(n,999)-v) for n,v in self.target.items() if 'gripper' in n)
            if gripper_error>.025:
                self.hold_since=None;return
            self.results.append(dict(cycle=self.cycle,success=True,placement_world_m=self.placement,home_verified=True,home_arm_max_error_rad=arm_error,home_gripper_max_error_rad=gripper_error,base_world_m=self.point('robomaster_ep').tolist(),heading_error_rad=angle(yaw(self.poses['robomaster_ep'].rotation)-self.base_yaw),duration_seconds=now-self.cycle_started))
            self.event('CYCLE_HOME_VERIFIED',completed_cycles=len(self.results));self.save_results('RUNNING')
            if self.cycle==5:
                self.active=False;self.velocity.publish(Twist());self.phase='FIVE_CYCLES_COMPLETED_AT_HOME'
                self.state.publish(String(data=self.phase));self.event(self.phase);self.save_results('COMPLETED');return
            self.enter('REFILL_WAIT',*self.cfg['home_tip_xz_m'],.6,1)
            if self.cfg['refill_mode']=='automatic_after_home':
                if not self.refill.service_is_ready():self.fail('REFILL_SERVICE_MISSING');return
                self.event('SCENE_REFILL_REQUESTED_AFTER_HOME',note='Scene preparation only; not a grasp or transport action')
                self.refill_future=self.refill.call_async(Trigger.Request())
            else:self.event('WAIT_FOR_MANUAL_REFILL_AT_A')

if __name__=='__main__':
    rclpy.init();n=Transfer()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.velocity.publish(Twist());n.destroy_node();rclpy.shutdown()
