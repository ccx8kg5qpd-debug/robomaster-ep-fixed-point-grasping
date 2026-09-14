from pathlib import Path
import json, os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
def generate_launch_description():
    root=Path(__file__).resolve().parent
    names=json.loads((root/'ep_joint_names.json').read_text())
    bridges=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
      '/ep/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
      '/ep/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
      '/world/pick_place_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
      '/world/pick_place_world/model/pick_box/link/box_link/sensor/box_contacts/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts']
    bridges += ['/ep/joint/'+n+'@std_msgs/msg/Float64]ignition.msgs.Double' for n in names]
    return LaunchDescription([
      SetEnvironmentVariable('IGN_IP','127.0.0.1'),
      SetEnvironmentVariable('PYTHONNOUSERSITE','1'),
      ExecuteProcess(cmd=['ign','gazebo',*(['-s'] if os.environ.get('EP_HEADLESS')=='1' else []),'-r',str(root/'ep_feedback.world.sdf')]),
      ExecuteProcess(cmd=['ros2','run','ros_gz_bridge','parameter_bridge',*bridges]),
      TimerAction(period=5.,actions=[ExecuteProcess(cmd=['ros2','run','ros_gz_sim','create','-name','robomaster_ep','-file',str(root/'ep_feedback.sdf'),'-z','.061'])]),
      ExecuteProcess(cmd=['/usr/bin/python3',str(root/'scene_refill.py')]),
      TimerAction(period=8.,actions=[ExecuteProcess(cmd=['/usr/bin/python3',str(root/'ep_transfer_control.py')])]),
    ])
