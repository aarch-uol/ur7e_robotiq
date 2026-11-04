
sensor_msgs.msg.JointState(
    header=std_msgs.msg.Header(
        stamp=builtin_interfaces.msg.Time(sec=1665564900, nanosec=238801330),
        frame_id=''), 
    name=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'rg2_finger_joint1', 'rg2_finger_joint2'],
    position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    effort=[nan, nan, nan, nan, nan, nan, nan, nan])

geometry_msgs.msg.TransformStamped(
    header=std_msgs.msg.Header(
        stamp=builtin_interfaces.msg.Time(sec=1665564543, nanosec=518767722),
    frame_id='base_link'), 
    child_frame_id='rg2_hand', 
    transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.0, y=0.19424999999999998, z=0.69415), 
    rotation=geometry_msgs.msg.Quaternion(x=0.5000000000000001, y=0.4999999991025518, z=0.5000000000000001, w=0.500000000897448)))