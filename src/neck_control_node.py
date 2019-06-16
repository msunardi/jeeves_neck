#!/usr/bin/env python
import sys, threading
import random as r
import rospy
from std_msgs.msg import Float32, Float64
from collections import deque
#from arbotix_msgs.srv import *

#from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_controllers.srv import SetComplianceSlope, TorqueEnable


class NeckController(threading.Thread):
    roll = 0.0
    pan = 0.0
    then = 0

    def __init__(self):
        self.yaw_sub = rospy.Subscriber('/head/cmd_pose_yaw', Float64, self.yaw_callback)
        self.pitch_sub = rospy.Subscriber('/head/cmd_pose_pitch', Float64, self.pitch_callback)
        self.pan_pub = rospy.Publisher('/neck/pan_controller/command', Float64, queue_size=10)
        self.tilt_left_pub = rospy.Publisher('/neck/tilt_left_controller/command', Float64, queue_size=10)
        self.tilt_right_pub = rospy.Publisher('/neck/tilt_right_controller/command', Float64, queue_size=10)
        #self.roll_pub = rospy.Publisher('/head_roll_joint/command', Float64, queue_size=10)

        #rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        #self.dynamixel_service = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)

	#rospy.wait_for_service('/pan_controller/relax')
	#rospy.wait_for_service('/tilt_left_controller/relax')
	#rospy.wait_for_service('/tilt_right_controller/relax')
	rospy.wait_for_service('/neck/pan_controller/torque_enable')
	rospy.wait_for_service('/neck/tilt_left_controller/torque_enable')
	rospy.wait_for_service('/neck/tilt_right_controller/torque_enable')
        rospy.wait_for_service('/neck/pan_controller/set_compliance_slope')
        rospy.wait_for_service('/neck/tilt_right_controller/set_compliance_slope')
        rospy.wait_for_service('/neck/tilt_left_controller/set_compliance_slope')
	#self.pan_relax = rospy.ServiceProxy('/pan_controller/relax', Relax)
	#self.tilt_left_relax = rospy.ServiceProxy('/tilt_left_controller/relax', Relax)
	#self.tilt_right_relax = rospy.ServiceProxy('/tilt_right_controller/relax', Relax)
	self.pan_enable = rospy.ServiceProxy('/neck/pan_controller/torque_enable', TorqueEnable)
	self.tilt_left_enable = rospy.ServiceProxy('/neck/tilt_left_controller/torque_enable', TorqueEnable)
	self.tilt_right_enable = rospy.ServiceProxy('/neck/tilt_right_controller/torque_enable', TorqueEnable)
        self.pan_slope = rospy.ServiceProxy('/neck/pan_controller/set_compliance_slope', SetComplianceSlope )
        self.tilt_left_slope = rospy.ServiceProxy('/neck/tilt_left_controller/set_compliance_slope', SetComplianceSlope )
        self.titl_right_slope = rospy.ServiceProxy('/neck/tilt_right_controller/set_compliance_slope', SetComplianceSlope )

        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(10)
        
    def setSlope(self, slope=255):
        try:
            rospy.info("Setting controller slopes to: {}".format(slope))
            rospy.wait_for_service('/neck/pan_controller/set_compliance_slope')
            rospy.wait_for_service('/neck/tilt_right_controller/set_compliance_slope')
            rospy.wait_for_service('/neck/tilt_left_controller/set_compliance_slope')
            self.pan_slope(slope)
            self.tilt_left_slope(slope)
            self.tilt_right_slope(slope)
        except Exception as e:
            rospy.logerr("Failed setting slope service. Reason: {}".format(e))

    def run(self):
        rospy.loginfo("Setting up neck controller...")
        self.setSlope()
        while not rospy.is_shutdown():
            dtime = rospy.get_time() - self.then
            if dtime > 2 + r.randint(1, 5):
                self.setSlope()
                self.adjust_roll()
                self.then = rospy.get_time()
            self.sleeper.sleep() 

    def yaw_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Got pan data: %s" % data.data)
        self.pan = data.data * 0.8
        self.roll = data.data * 0.3
        self.pan_pub.publish(Float64(self.pan))

        #self.dynamixel_service('', 3, self.pan, 
        #self.roll_pub.publish(Float64(self.roll))

    def pitch_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Got pitch data: %s" % data.data)
        tilt = data.data * -0.8
        self.tilt_pub.publish(Float64(tilt))

    def adjust_roll(self):
        x = r.random()
        if x < 0.4:
            self.roll += r.random() * self.pan
            self.roll = min(self.roll, 0.8)
            self.tilt_right_pub.publish(Float64(self.roll))
        elif x >= 0.4 and x < 0.8:
            self.roll -= r.random() * self.pan
            self.roll = max(self.roll, -0.8)
            self.tilt_left_pub.publish(Float64(self.roll))
        else:
            z = r.random()
            if z < 0.7: 
                # Reset head roll position
                self.tilt_right_pub.publish(Float64(0.0))
                self.tilt_left_pub.publish(Float64(0.0))
                return None
        # check current position
        #self.tilt_left_pub.publish(Float64(self.roll))
        #self.tilt_right_pub.publish(
            

def main(args):
    rospy.init_node('neck_controller_node', anonymous=True)
    necker = NeckController()
    necker.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)

