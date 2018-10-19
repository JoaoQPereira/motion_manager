#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
import mpl_toolkits.axes_grid1
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs.msg import JointState

# ----------------------------------------------------------------------------------------------------------- #
#                                              Class JointStates                                              #
# ----------------------------------------------------------------------------------------------------------- #
class Hand:
    # ***************** #
    #   FUNCTION: init  #
    # ***************** #
    def __init__(self):
        # Initialization of variables
        self.joints_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6']
        self.joints_pos = []
        self.joints_vel = []
        self.nJoints = 7
        self.iTime = 0
        self.StopPlot = 0
        self.hand_pos = []

        # Topic to subcribe to
        rospy.Subscriber("/robot/joint_states", JointState, self.callbackJoints)
        rate = rospy.Rate(10) # Hz

        while not rospy.is_shutdown():
            # Get hand position and plot data
            self.getHandPos()
            self.plotData()
            rate.sleep()

    # *************************** #
    #   FUNCTION: callbackJoints  #
    # *************************** #
    def callbackJoints(self, data):
        pos = []
        vel = []

        for allJoints_name in data.name:
            if allJoints_name in self.joints_names:
                # Get joint index
                i = data.name.index(allJoints_name)
                # Get joint position
                pos.insert(i, data.position[i])
                # Get joint velocity
                vel.insert(i, data.velocity[i])

        # Save the position and velocity of the joints
        self.joints_pos = pos
        self.joints_vel = vel

    # ********************* #
    #   FUNCTION: plotData  #
    # ********************* #
    def plotData(self):
        print self.hand_pos

    # *********************** #
    #   FUNCTION: getHandPos  #
    # *********************** #
    def getHandPos(self):
        # DH parameters
        dH_a = [0, 0.081, 0, 0, 0, 0, 0]
        dH_alpha = [0, -90, -90, 90, -90, 90, -90]
        dH_d = [0.317, 0.1925, 0.4, 0.1685, 0.4, 0.1363, 0.2652]
        # Identity matrix
        T = np.identity(4)
        # Hand position
        handPos = []

        if self.joints_pos:
            for x in range(0, self.nJoints):
                # Transformation matrix between the joints of the manipulator
                T_aux = self.transfMatix(dH_a[x], dH_alpha[x], dH_d[x], self.joints_pos[x])
                # Transformation matrix between each of the joints and the base of the manipulator
                T = [[sum(a * b for a, b in zip(T_row, T_aux_col)) for T_aux_col in zip(*T_aux)] for T_row in T]

            # Get and save the position in x, y and z from the hand of the manipulator
            handPos.append(T[0][3])
            handPos.append(T[1][3])
            handPos.append(T[2][3])
            # Save the hand position
            self.hand_pos = handPos

    # ************************ #
    #   FUNCTION: transfMatix  #
    # ************************ #
    def transfMatix(self, a, alpha, d, theta):
        T_aux = np.identity(4)
        # First row
        T_aux[0][0] = np.cos(theta)
        T_aux[0][1] = -np.sin(theta)
        T_aux[0][2] = 0.0
        T_aux[0][3] = a
        # Second row
        T_aux[1][0] = np.sin(theta) * np.cos(alpha)
        T_aux[1][1] = np.cos(theta) * np.cos(alpha)
        T_aux[1][2] = -np.sin(alpha)
        T_aux[1][3] = -np.sin(alpha) * d
        # Third row
        T_aux[2][0] = np.sin(theta) * np.sin(alpha)
        T_aux[2][1] = np.cos(theta) * np.sin(alpha)
        T_aux[2][2] = np.cos(alpha)
        T_aux[2][3] = np.cos(alpha) * d
        # Fourth row
        T_aux[3][0] = 0.0
        T_aux[3][1] = 0.0
        T_aux[3][2] = 0.0
        T_aux[3][3] = 1.0

        return T_aux


    # ****************** #
    #   FUNCTION: bStop  #
    # ****************** #
    def bStop(self, event):
        self.StopPlot = 1

    # ******************* #
    #   FUNCTION: bClean  #
    # ******************* #
    def bClear(self, event):
        # Reset all variables used in the plot
        pass

    # ******************* #
    #   FUNCTION: bStart  #
    # ******************* #
    def bStart(self, event):
        self.StopPlot = 0

# ----------------------------------------------------------------------------------------------------------- #
#                                                    Main                                                     #
# ----------------------------------------------------------------------------------------------------------- #
def main():
    rospy.init_node('monitoring_interface_hand')
    listener = Hand()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
