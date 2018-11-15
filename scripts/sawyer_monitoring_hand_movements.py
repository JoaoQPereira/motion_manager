#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
import mpl_toolkits.axes_grid1
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import proj3d
from sensor_msgs.msg import JointState
from mpl_toolkits import mplot3d
from numpy import linalg as LA

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
        self.nJoints = 7
        self.handPos = []
        self.handVel = 0.0
        # DH parameters
        self.dH_a = [0, 81, 0, 0, 0, 0, 0]
        self.dH_alpha = [0, -1.5707963268, -1.5707963268, 1.5707963268, -1.5707963268, 1.5707963268, -1.5707963268]
        self.dH_d = [317, 192.5, 400, -168.5, 400, 136.3, 265.2]
        # World Transformation Matrix (Base of the arm relative to the floor)
        self.T_world = self.transMatrixInd(-0.00033700562527, -0.00000011784288577, 3.1415926536, [49.988, 14.982, 910.0])

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

        self.handPos = self.getHandPos()
        self.handVel = self.getHandVel(self.handPos)

    # *********************** #
    #   FUNCTION: getHandPos  #
    # *********************** #
    def getHandVel(self, handPos):
        # Jacobian Matrix
        jacobian = np.zeros((6,7));
        z = [0] * 3
        pos = [0] * 3
        diff = [0] * 3
        # Transformation Matrix
        T = self.T_world

        if self.joints_pos and self.joints_vel:
            #
            jointVel = self.joints_vel

            #
            for a, alpha, d, theta, joint in zip(self.dH_a, self.dH_alpha,
                                                 self.dH_d, self.joints_pos, range(self.nJoints)):
                # Transformation matrix between the joints of the manipulator
                if joint == 1:
                    T_aux = self.transfMatix(a, alpha, d, theta - 1.5707963268)
                else:
                    T_aux = self.transfMatix(a, alpha, d, theta)
                # Transformation matrix between each of the joints and the base of the manipulator
                T = [[sum(a * b for a, b in zip(T_row, T_aux_col)) for T_aux_col in zip(*T_aux)] for T_row in T]

                #
                for item, hand, aux in zip(T, handPos, range(0,3)):
                    if aux != 3:
                        z[aux] = item[2]
                        pos[aux] = item[3]
                        diff[aux] = hand - item[3]

                #
                cross = np.cross(z, diff)
                #
                column = np.concatenate((cross, z), axis = None)
                #print 'joint: {}'.format(joint)
                #print 'column: {}'.format(column)
                #print '***************************************************'
                #
                for item, value in zip(jacobian, column):
                    item[joint] = value

            #
            #print 'hand pos: {}'.format(handPos)
            handVel = np.tensordot(jacobian, jointVel, axes = 1)
            handVelLinear = [handVel[0], handVel[1], handVel[2]]
            handVelLinearNorm = LA.norm(handVelLinear)

            #print 'JOINT: {}'.format(joint)
            #print 'T: {}'.format(T)
            #print 'z: {}'.format(z)
            #print 'pos: {}'.format(pos)
            #print 'diff: {}'.format(diff)
            #print 'cross: {}'.format(cross)
            #print 'column: {}'.format(column)
            #print 'jacobian: {}'.format(jacobian)
            #print 'jointsVel: {}'.format(jointVel)
            #print 'handVel: {}'.format(handVel)
            #print 'handVelLinear: {}'.format(handVelLinear)

            return handVelLinearNorm

    # *********************** #
    #   FUNCTION: getHandPos  #
    # *********************** #
    def getHandPos(self):
        # Hand Position
        hand_pos = []
        # Joint Number
        joint = 0
        # Transformation Matrix
        T = self.T_world

        if self.joints_pos:
            for a, alpha, d, theta in zip(self.dH_a, self.dH_alpha, self.dH_d, self.joints_pos):
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
        vel = []
        t = []

        # *-*-*-*-*-*-*-*-*-*- #
        #   Create the figure  #
        fig = plt.figure()
        # Maximize the figure
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        # Title of the figure
        fig.suptitle('[Sawyer Robot]: Monitoring of Hand Movements', fontsize = 35, fontweight = 'bold', color = 'peru')

        # *-*-*-*-*-*-*-*-*-*-*-*-*- #
        #   Initialize the subplots  #
        # Position
        gs = gridspec.GridSpec(3,1)
        axisPos = fig.add_subplot(gs[0:2], projection = '3d')
        axisVel = fig.add_subplot(gs[-1])
        axisPos.view_init(5, 25)

        # ******************* #
        #   FUNCTION: update  #
        # ******************* #
        def update(i):
            if self.handPos:
                # *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
                #          Hand Position          #
                # *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
                for position, value in zip(pos, self.handPos):
                    position.append(value)

                # *-*-*-*-*-*-*-*- #
                #   Plot the data  #
                # Configurations
                axisPos.clear()
                axisPos.set_title('Hand Position', fontweight = 'bold', fontsize = 18)
                axisPos.set_xlabel('y [mm]', fontsize = 10)
                axisPos.set_ylabel('x [mm]', fontsize = 10)
                axisPos.set_zlabel('z [mm]', fontsize = 10)
                axisPos.set_xlim([min(pos[1]) - 10, max(pos[1]) + 10])
                axisPos.set_ylim([min(pos[0]) + 10, max(pos[0]) - 10])
                axisPos.set_zlim([min(pos[2]) - 10, max(pos[2]) + 10])

                for i in range(0, len(pos[0])):
                    # Start Point = Start Posiion of the Arm
                    if i == 0:
                        axisPos.scatter(pos[1][i], pos[0][i], pos[2][i], color = 'cornflowerblue',
                                        marker = '^', s = 175, alpha = 1.0)
                        x2D, y2D, _ = proj3d.proj_transform(pos[1][i], pos[0][i], pos[2][i], axisPos.get_proj())
                        axisPos.annotate('Start', xy = (x2D, y2D), xytext = (-30, 0),
                                         textcoords = 'offset points', ha = 'right', va = 'bottom',
                                         bbox = dict(boxstyle = 'round, pad = 0.5', fc = 'cornflowerblue', alpha = 0.45),
                                         arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3, rad = 0'))
                    # Current Point = Current Position of the arm
                    elif i == len(pos[0]) - 1:
                        axisPos.scatter(pos[1][i], pos[0][i], pos[2][i], color = 'forestgreen',
                                        marker = 'v', s = 175, alpha = 1.0)
                        x2D, y2D, _ = proj3d.proj_transform(pos[1][i], pos[0][i], pos[2][i], axisPos.get_proj())
                        axisPos.annotate('Current', xy = (x2D, y2D), xytext = (80, 0),
                                         textcoords = 'offset points', ha = 'right', va = 'bottom',
                                         bbox = dict(boxstyle = 'round, pad = 0.5', fc = 'forestgreen', alpha = 0.45),
                                         arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3, rad = 0'))
                    # All other points in the trajectory
                    else:
                        axisPos.scatter(pos[1][i], pos[0][i], pos[2][i], color = 'firebrick',
                                        marker = '.', s = 170, alpha = 0.50)

            # *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
            #          Hand Velocity          #
            # *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #'
            if self.handVel:
                # Save hand velocity
                vel.append(self.handVel)
                # Save current time
                if self.iTime == 0:
                    t.append(0.0)
                    self.iTime = time.clock()
                else:
                    t.append(time.clock() - self.iTime)

                # *-*-*-*-*-*-*-*- #
                #   Plot the data  #
                # Configurations
                axisVel.clear()
                axisVel.set_title('Hand Velocity', fontweight = 'bold', fontsize = 18)
                axisVel.set_xlabel('Time [s]', fontsize = 11)
                axisVel.set_ylabel('[mm/s]', fontsize = 11)
                axisVel.set_xlim([max(0, t[-1] - 40), t[-1] + 0.0001])
                axisVel.set_ylim([0.0, max(vel) + 5])
                axisVel.spines['left'].set_color('darkgoldenrod')
                axisVel.spines['left'].set_linewidth(3)
                # Plot the velocity
                axisVel.plot(t, vel, color = 'darkgoldenrod', linewidth = 1.65, label = 'Vel [mm/s]')

        a = anim.FuncAnimation(fig, update, frames = 1000)
        plt.show()

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
