from controllerGui import *
import sys
from PyQt5.QtWidgets import *

from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2Servo, MoveIt2ServoJoints
from pymoveit2.robots import ur3

from sensor_msgs.msg import JointState
from example_interfaces.msg import Float64



#Executes GUI
app = QtWidgets.QApplication(sys.argv)
MainWindow = QtWidgets.QMainWindow()
ui = Ui_MainWindow()
ui.setupUi(MainWindow)
#MainWindow.show()

# Initialize ROS

 

def servo_on_off():
    if ui.checkBox.isChecked():
        if ui.radioButton.isChecked():
            servo_init()
        else:
            try:
                destroy()
                servoj_init()
            except:
                servoj_init() 
    else:
        destroy
# Locks ComboBox when ServoJ enabled
def lock_joint_frame():
    ui.comboBox.setCurrentIndex(0)
    ui.comboBox.setDisabled(1)
# Unlocks ComboBox when ServoJ disabled
def unlock_joint_frame():
    ui.comboBox.setEnabled(1)
# Changes publisher to Servo
def radio_button_change_1(self):
    if ui.checkBox.isChecked():
        self.destroy()
        self.servo_init()
    else:
        self.servo_init()
# Changes publisher to ServoJ
def radio_button_change_2(self):
    if ui.checkBox.isChecked():
        self.destroy()
        self.servoj_init() 



def destroy():
    try:
        node.destroy_node()
    except:
        print("Failed")

def pose(self):
    try:
        node = Node("ex_pose_goal")
        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
            node=node,
            joint_names=ur3.joint_names(),
            base_link_name=ur3.base_link_name(),
            end_effector_name=ur3.end_effector_name(),
            group_name=ur3.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        executor.add_node(node)
        position = [ui.doubleSpinBox.value(), ui.doubleSpinBox_2.value(), ui.doubleSpinBox_3.value()] 
        quat_xyzw = [ui.doubleSpinBox_4.value(), ui.doubleSpinBox_5.value(), ui.doubleSpinBox_6.value(), ui.doubleSpinBox_7.value()]
        node.get_logger().info(
            "Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw)
        moveit2.wait_until_executed()
        node.destroy_node()
    except:
        print("Failed to go Pose") 
        node.destroy_node()
           

def joint(self):
    try:
        node = Node("ex_joint_goal")
        node.declare_parameter(
            "joint_positions",
            [
            ui.spinBox.value()*22/1260,
            ui.spinBox_2.value()*22/1260,
            ui.spinBox_3.value()*22/1260,
            ui.spinBox_4.value()*22/1260,
            ui.spinBox_5.value()*22/1260,
            ui.spinBox_6.value()*22/1260,
            ],
        )
        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
            node=node,
            joint_names=ur3.joint_names(),
            base_link_name=ur3.base_link_name(),
            end_effector_name=ur3.end_effector_name(),
            group_name=ur3.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.executor.add_node(node)
        joint_positions = (
            node.get_parameter("joint_positions").get_parameter_value().double_array_value
        )
        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
        moveit2.move_to_configuration(joint_positions)
        moveit2.wait_until_executed()
        node.destroy_node()
    except:
        print("Failed to MoveJ")
        node.destroy_node()

def servoj_init(self):
    self.node = Node("ex_servoj")
    callback_group = ReentrantCallbackGroup()
    self.moveit2_servoj = MoveIt2ServoJoints(
        node=self.node,
        frame_id= ur3.base_link_name(),
        joint_names=ur3.joint_names(),  #["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"],
        callback_group=callback_group,
    )
     
    executor.add_node(self.node)
    self.node.create_timer(0.01,self.servoj)

# Define SpinBoxes for Servo  
def servoj(self):
    self.moveit2_servoj.servoJ(
        velocities=(ui.spinBox_7.value()/100, ui.spinBox_8.value()/100, ui.spinBox_9.value()/100,
        ui.spinBox_10.value()/100, ui.spinBox_11.value()/100, ui.spinBox_12.value()/100)) 
    print(float(ui.spinBox_7.value()))
    
def servo_init(self):
    if ui.checkBox.isChecked(): 
        self.node = Node("ex_servo")
        callback_group = ReentrantCallbackGroup()
        if ui.comboBox.currentIndex() == 0:
            self.moveit2_servo = MoveIt2Servo(
              node=self.node,
              frame_id=ur3.base_link_name(),
              callback_group=callback_group,
              linear_speed=0.01,
              angular_speed=0.01,
            )
        elif ui.comboBox.currentIndex() == 1:
            self.moveit2_servo = MoveIt2Servo(
              node=self.node,
              frame_id=ur3.end_effector_name(),
              callback_group=callback_group,
              linear_speed=0.01,
              angular_speed=0.01,
            )         
        self.executor.add_node(self.node)
        self.node.create_timer(0.01,self.servo)
    else:
        print("ERROR")
    
# Define SpinBoxes for Servo    
def servo(self):
    self.moveit2_servo.servo(
        linear=(ui.spinBox_7.value(), ui.spinBox_8.value(), ui.spinBox_9.value()), 
        angular=(ui.spinBox_10.value(), ui.spinBox_11.value(), ui.spinBox_12.value()))
    print(float(ui.spinBox_7.value()))



#Exit
#sys.exit(app.exec_())