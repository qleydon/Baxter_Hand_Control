#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy
import tf
import rospy
import rospkg
import time
import math

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface



class PickAndPlace(object):
    def __init__(self, limb, verbose=True):
        self._limb_name = limb # string
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
            #    print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            #self._limb.move_to_joint_positions(joint_angles)
	    self._limb.move_to_joint_positions(joint_angles, threshold= 0.1)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _current_pos(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)
        
    def to_pose(self, pose):
        self._servo_to_pose(pose)
        
class q_odom:
  def __init__(self):
    self.Pos_data = [0,0,0,0,0,0,0]
    self.pose_subscriber = rospy.Subscriber('/my_Pos', Pose, self.callback)
    
  def get_Pos_data(self):
        return self.Pos_data
    
  def callback(self, data):
    self.Pos_data = [float(data.position.x), float(data.position.y), float(data.position.z),
    float(data.orientation.x),float(data.orientation.y),
    float(data.orientation.z),float(data.orientation.w)]
    time.sleep(1)
    
    

def pos_from_rpy(rpy):
    quat = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], 'rxyz')
    q = Quaternion(
                 x=quat[0],
                 y=quat[1],
                 z=quat[2],
                 w=quat[3])
    return q

def pos_from_quat(quat):
    q = Quaternion(
             x=quat[0],
             y=quat[1],
             z=quat[2],
             w=quat[3])
    print(quat)
    print(q)
    return q

def main():
    # ---------------------------------------------------------------------------------------
    # Choose input type:
    # 0 = Constant angle, Constant position
    # 1 = Constant angle, Changing position
    # 2 = Constant angle, Changing position
    # 3 = Changing angle, Changing posiiton
    input_type = 0
    # ---------------------------------------------------------------------------------------
    c = (2*math.pi / 360)
    ang = [15*c,90*c,0*c]
    coords= Point(x=0.7, y=0.5, z=0.3)
    # ---------------------------------------------------------------------------------------
    # Choose RPY_type
    # 0 = No Yaw
    # 1 = No Roll
    RPY_type = 1
    # ---------------------------------------------------------------------------------------
    limb = 'left'
    #limb = 'right'
    # ---------------------------------------------------------------------------------------
    
    rospy.init_node("ik_pick_and_place_demo")
    rbt = q_odom()
    # Wait for the All Clear from emulator startup
    #rospy.wait_for_message("/robot/sim/started", Empty)
    #ang = [3,0,-3]
    pnp = PickAndPlace(limb)
    # Each additional pose will get its own pick and place.
    poses=Pose(
        position=coords,
        orientation = pos_from_rpy(ang))
        
    idx = 0
    while not rospy.is_shutdown():
        PD = rbt.get_Pos_data()
        rpy = PD[3:6]
        quat = PD[3:7]
        #rpyc = [PD[0]*c, PD[1]*c, PD[2]*c]
        rpyc = [PD[0]*c, PD[1]*c, 0]
        if RPY_type == 0:
            ori = pos_from_rpy(rpyc)
        elif RPY_type == 1:
            ori = pos_from_quat(quat)
        # ----------------------------------------------
        if input_type == 0:
            poses=Pose(
                position=coords,
                orientation = pos_from_rpy(ang))
        elif input_type == 1:
            poses=Pose(
                position=Point(x=PD[0], y=PD[1], z=PD[2]),
                orientation = pos_from_rpy(ang))
        elif input_type == 2:
            poses=Pose(
                position=coords,
                orientation = ori)
        elif input_type == 3:
            poses=Pose(
                position=Point(x=PD[0], y=PD[1], z=PD[2]),
                orientation = ori)
        # ----------------------------------------------
        pnp.to_pose(poses)
        time.sleep(0.5)
    return 0

if __name__ == '__main__':
    sys.exit(main())
    
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """

