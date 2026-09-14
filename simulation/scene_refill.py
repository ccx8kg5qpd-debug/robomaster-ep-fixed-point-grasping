"""Scene preparation between verified cycles. Never invoked during grasp/transport."""
import json
import subprocess
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class Refill(Node):
    def __init__(self):
        super().__init__('ep_scene_refill')
        self.phase=''
        self.create_subscription(String,'/pick_place/state',self.state,10)
        self.create_service(Trigger,'/pick_place/refill_at_home',self.refill)
    def state(self,msg):self.phase=msg.data
    def refill(self,req,res):
        if self.phase!='REFILL_WAIT':
            res.success=False;res.message='Refill permitted only after home verification';return res
        cfg=json.loads((Path(__file__).parent/'experiment.json').read_text())
        x,y,z=cfg['a_world_m']
        request=f'name: "pick_box", position: {{x: {x}, y: {y}, z: {z+.001}}}, orientation: {{w: 1, x: 0, y: 0, z: 0}}'
        try:
            p=subprocess.run(['ign','service','-s','/world/pick_place_world/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','3000','--req',request],capture_output=True,text=True,timeout=5)
            res.success=p.returncode==0 and 'data: true' in p.stdout
            res.message=p.stdout+p.stderr
        except Exception as e:res.success=False;res.message=str(e)
        self.get_logger().info('SCENE_REFILL '+str(res.success));return res

if __name__=='__main__':
    rclpy.init();node=Refill()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
