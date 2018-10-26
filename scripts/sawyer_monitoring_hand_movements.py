#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
import mpl_toolkits.axes_grid1
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from sensor_msgs.msg import JointState
from mpl_toolkits import mplot3d

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
        self.iTime = 0
        self.StopPlot = 0

        # Topic to subcribe to
        rospy.Subscriber("/robot/joint_states", JointState, self.callbackJoints)
        rate = rospy.Rate(10) # Hz

        while not rospy.is_shutdown():
            if self.joints_pos:
                self.plotHandPos()
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

    # *********************** #
    #   FUNCTION: getHandPos  #
    # *********************** #
    def getHandPos(self):
        # DH parameters
        dH_a = [0, 81, 0, 0, 0, 0, 0]
        dH_alpha = [0, -1.5707963268, -1.5707963268, 1.5707963268, -1.5707963268, 1.5707963268, -1.5707963268]
        dH_d = [317, 192.5, 400, -168.5, 400, 136.3, 265.2]
        # World Transformation Matrix (Base of the arm relative to the floor)
        T = self.transMatrixInd(-0.00033700562527, -0.00000011784288577, 3.1415926536, [49.988, 14.982, 910.0])
        # Hand Position
        hand_pos = []
        # Joint Number
        joint = 0

        if self.joints_pos:
            for a, alpha, d, theta in zip(dH_a, dH_alpha, dH_d, self.joints_pos):
                # Transformation matrix between the joints of the manipulator
                if joint == 1:
                    T_aux = self.transfMatix(a, alpha, d, theta - 1.5707963268)
                else:
                    T_aux = self.transfMatix(a, alpha, d, theta)
                # Transformation matrix between each of the joints and the base of the manipulator
                T = [[sum(a * b for a, b in zip(T_row, T_aux_col)) for T_aux_col in zip(*T_aux)] for T_row in T]
                joint = joint + 1

            # Get and save the position in x, y and z from the hand of the manipulator
            hand_pos.append(T[0][3])
            hand_pos.append(T[1][3])
            hand_pos.append(T[2][3])

        return hand_pos

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

    # *************************** #
    #   FUNCTION: trasnMatrixInd  #
    # *************************** #
    def transMatrixInd(self, alpha, beta, gama, Pos):
        T = np.identity(4)
        Rot = self.rpyMatrix(alpha, beta, gama)
        # First row
        T[0][0] = Rot[0][0]
        T[0][1] = Rot[0][1]
        T[0][2] = Rot[0][2]
        T[0][3] = Pos[0]
        # Second row
        T[1][0] = Rot[1][0]
        T[1][1] = Rot[1][1]
        T[1][2] = Rot[1][2]
        T[1][3] = Pos[1]
        # Third row
        T[2][0] = Rot[2][0]
        T[2][1] = Rot[2][1]
        T[2][2] = Rot[2][2]
        T[2][3] = Pos[2]
        # Fourth row
        T[3][0] = 0.0
        T[3][1] = 0.0
        T[3][2] = 0.0
        T[3][3] = 1.0

        return T

    # ********************** #
    #   FUNCTION: rpyMatrix  #
    # ********************** #
    def rpyMatrix(self, alpha, beta, gama):
        RPY = np.identity(3)
        # First row
        RPY[0][0] = np.cos(gama) * np.cos(beta)
        RPY[0][1] = np.cos(gama) *  np.sin(beta) * np.sin(alpha) - np.sin(gama) * np.cos(alpha)
        RPY[0][2] = np.sin(gama) *  np.sin(alpha) + np.cos(gama) * np.sin(beta) * np.cos(alpha)
        # Second row
        RPY[1][0] = np.sin(gama) * np.cos(beta)
        RPY[1][1] = np.cos(gama) *  np.cos(alpha) + np.sin(gama) * np.sin(beta) * np.sin(alpha)
        RPY[1][2] = np.sin(gama) *  np.sin(beta) * np.cos(alpha) - np.cos(gama) * np.sin(alpha)
        # Third row
        RPY[2][0] = - np.sin(beta)
        RPY[2][1] = np.cos(beta) * np.sin(alpha)
        RPY[2][2] = np.cos(beta) * np.cos(alpha)

        return RPY

    # ************************ #
    #   FUNCTION: plotHandPos  #
    # ************************ #
    def plotHandPos(self):
        # Initialization of variables
        pos = [[] for i in range(0, 3)]
        # *-*-*-*-*-*-*-*-*-*- #
        #   Create the figure  #
        fig = plt.figure()
        # Maximize the figure
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        # Title of the figure
        fig.suptitle('[Sawyer Robot]: Monitoring of Hand Movements', fontsize = 35, fontweight = 'bold', color = 'peru')
        # Initialize the subplots of the figure
        axisPos = fig.add_subplot(1, 1, 1, projection = '3d')

        # ******************* #
        #   FUNCTION: update  #
        # ******************* #
        def update(i):
            # Get data
            handPos = self.getHandPos()
            if handPos:
                for position, value in zip(pos, handPos):
                    position.append(value)

                # Plot data
                axisPos.clear()
                axisPos.set_title('Hand Position', fontweight = 'bold', fontsize = 18)
                axisPos.set_xlabel('x [mm]', fontsize = 11)
                axisPos.set_xlim([min(pos[0]) - 5, max(pos[0]) + 5])
                axisPos.set_ylabel('y [mm]', fontsize = 11)
                axisPos.set_ylim([min(pos[1]) - 5, max(pos[1]) + 5])
                axisPos.set_zlabel('z [mm]', fontsize = 11)
                axisPos.set_zlim([min(pos[2]) - 5, max(pos[2]) + 5])

                for i in range(0, len(pos[0])):
                    # All Other Points
                    if i != 0 and i != len(pos[0]) - 1:
                        co = 'firebrick'
                        ma = 'o'
                        si = 75
                        al = 0.5
                    # Initial Point
                    elif i == 0:
                        co = 'seagreen'
                        ma = '^'
                        si = 175
                        al = 1.0
                    # Final Point
                    else:
                        co = 'slateblue'
                        ma = 'v'
                        si = 175
                        al = 1.0

                    axisPos.scatter(pos[0][i], pos[1][i], pos[2][i], color = co, marker = ma, s = si, alpha = al)
                    
        a = anim.FuncAnimation(fig, update, frames = 1000)
        plt.show()

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
