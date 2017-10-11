#!/usr/bin/env python
#spins off a thread to listen for joint_states messages
#and provides the same information (or subsets of) as a service

import rospy
from robotino_msgs.srv import *
from sensor_msgs.msg import JointState
import threading


#holds the latest states obtained from joint_states messages
class LatestJointStates:

    def __init__(self):
        rospy.init_node('joint_states_listener')
        self.lock = threading.Lock()
        self.joints = {}
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

        s = rospy.Service('return_joint_states', ReturnJointStates, self.return_joint_states)
        

    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.spin()


    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.lock.acquire()
        for idx, val in enumerate(msg.name):
            try:
                self.joints[val] = (msg.position[idx], msg.velocity[idx], msg.effort[idx])
            except IndexError:
                rospy.logdebug('Unable to set velocity or effort')
                self.joints[val] = (msg.position[idx], 0.0, 0.0)
                
        self.lock.release()

    #returns (found, position, velocity, effort) for the joint joint_name 
    #(found is 1 if found, 0 otherwise)
    def return_joint_state(self, joint_name):

        #return info for this joint
        self.lock.acquire()
        for i in self.joints:
            rospy.loginfo('names: {}'.format(i))
        try: 
            rospy.loginfo(self.joints[joint_name])
            (position, velocity, effort) = self.joints[joint_name] 
            ret = (1, position, velocity, effort)
        except KeyError as e:
            rospy.logerr("Joint %s not found!", (joint_name))
            ret = (0, 0., 0., 0.)
        finally:
            self.lock.release()
        return ret


    #server callback: returns arrays of position, velocity, and effort
    #for a list of joints specified by name
    def return_joint_states(self, req):
        joints_found = []
        positions = []
        velocities = []
        efforts = []
        rospy.loginfo('{}'.format(req))
        for joint_name in req.name:
            (found, position, velocity, effort) = self.return_joint_state(joint_name)
            joints_found.append(found)
            positions.append(position)
            velocities.append(velocity)
            efforts.append(effort)
        return ReturnJointStatesResponse(joints_found, positions, velocities, efforts)


#run the server
if __name__ == "__main__":

    latestjointstates = LatestJointStates()

    print "joints_states_listener server started, waiting for queries"
    rospy.spin()
