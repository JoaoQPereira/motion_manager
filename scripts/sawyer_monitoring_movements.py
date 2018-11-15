#!/usr/bin/env python
import rospy
import os
import errno
import time
import numpy as np
import math
import Tkinter as tk
import ttk
import tkMessageBox
import tkFont
import matplotlib.gridspec as gridspec
import matplotlib.animation as anim
from mpl_toolkits.mplot3d import proj3d
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from sensor_msgs.msg import JointState
from datetime import datetime as dt
from numpy import linalg as LA


# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
#                                                                                                                                    #
#                                                         Class JointStates                                                          #
#                                                                                                                                    #
# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
class JointStates:
    # ************************************************ #
    #                  FUNCTION: init                  #
    # ************************************************ #
    def __init__(self):
        # Configuration variables
        self.joints_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6']
        self.nJoints = 7
        self.run = 0
        self.stop = 0
        self.save = 0
        # Create files to store the joints position and velocity over time
        self.configFiles()


    # ************************************************ #
    #              FUNCTION: subscribeNode             #
    # ************************************************ #
    def subscribeTopic(self, time):
        if not self.run:
            # ROS subscriber to the topic "/robot/joints_states"
            self.sub = rospy.Subscriber("/robot/joint_states", JointState, self.callbackJoints)
            # Set values of configuration variables
            self.run = 1
            self.save = 1
            self.tStart = time
            self.refTime = self.tStart
            self.severalCallback = 0
            # Open files to store the joints position and velocity
            self.fPos = open(self.filePos, 'a')
            self.fVel = open(self.fileVel, 'a')


    # ************************************************ #
    #              FUNCTION: subscribeNode             #
    # ************************************************ #
    def unsubscribeTopic(self):
        if self.run:
            # Set values of configuration variables
            self.save = 0
            self.run = 0
            # Close files
            self.fPos.close()
            self.fVel.close()


    # ************************************************ #
    #               FUNCTION: clearFiles               #
    # ************************************************ #
    def clearFiles(self):
        # Clean files to store the joints position and velocity
        self.fPos = open(self.filePos, 'w').close()
        self.fVel = open(self.fileVel, 'w').close()
        # Write the first two lines of the files: (i) the file name and (ii) the stored variables and their units of measure
        self.configFiles()


    # ************************************************ #
    #             FUNCTION: callbackJoints             #
    # ************************************************ #
    def callbackJoints(self, data):
        pos = []
        vel = []

        # Time elapsed since the last callback
        elapsedTime = (dt.now() - self.refTime).total_seconds()

        # Get and save the joints values, if the elapsed time is greater that 0.075 seconds
        if self.save and elapsedTime >= 0.25:
            for allJoints_name in data.name:
                # Finds the joints name to save
                if allJoints_name in self.joints_names:
                    # Joint's index
                    i = data.name.index(allJoints_name)
                    # Get the joint position
                    pos.insert(i, data.position[i])
                    # Get the joint velocity
                    vel.insert(i, data.velocity[i])

            if len(pos) == self.nJoints and len(vel) == self.nJoints:
                # Determines the time elapsed since the beginning of the movement
                t = (dt.now() - self.tStart).total_seconds()
                # If this is the first time that the callback is called, it stores the elapsed time
                if not self.severalCallback: self.firstTime = t
                t = t - self.firstTime
                self.refTime = dt.now()

                # Save the time and the joints position in the file
                self.fPos.write('{}, {} \n'.format(t, str(pos)[1:-1]))
                # Save the time and the joints velocity in the file
                self.fVel.write('{}, {} \n'.format(t, str(vel)[1:-1]))
                # Increment the auxiliary variable (mumber of times the callback is called)
                self.severalCallback += 1


    # ************************************************ #
    #               FUNCTION: configFiles              #
    # ************************************************ #
    def configFiles(self):
        # Files names
        self.filePos = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_pos.text'
        self.fileVel = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_vel.text'
        filesNames = [self.filePos, self.fileVel]

        for file in filesNames:
            # If the file doesn't exist, it is created.
            if not os.path.exists(os.path.dirname(file)):
                try:
                    os.makedirs(os.path.dirname(file))
                except OSError as exc:
                    if exc.errno != errno.EEXIST:
                        raise

            # Open the file
            with open(file, 'w') as f:
                # In the position file: we write the file name and the variables with the units measure
                # (Time in seconds and Joints Position in radians)
                if file == self.filePos:
                    f.write('# JOINTS POSITION \n')
                    f.write('# Time [s], J0 [rad], J1 [rad], J2 [rad], J3 [rad], J4 [rad], J5 [rad], J6 [rad]\n')
                # In the velocity file: we write the file name and the variables with the units measure
                # (Time in seconds and Joints Velocity in radians por seconds)
                else:
                    f.write('# JOINTS VELOCITY \n')
                    f.write('# Time [s], J0 [rad/s], J1 [rad/s], J2 [rad/s], J3 [rad/s], J4 [rad/s], J5 [rad/s], J6 [rad/s]\n')
                # Close the file
                f.close()



# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
#                                                                                                                                    #
#                                                      Class PlotJointsResults                                                       #
#                                                                                                                                    #
# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
class PlotJointsResults:
    # ************************************************ #
    #                  FUNCTION: init                  #
    # ************************************************ #
    def __init__(self, root):
        # Configuration variables
        self.nJoints = 7
        self.axisPos = [[] for i in range(self.nJoints)]
        self.axisVel = [[] for i in range(self.nJoints)]
        # Get the joints position and velocity to plot
        self.saveData()

        # Window and figure settings
        jointsWindow = tk.Toplevel(root)
        fig = Figure(figsize = (15, 10), facecolor = 'gainsboro')
        fig.subplots_adjust(top = 0.89, hspace = 0.55, wspace = 0.30)

        # Figure title
        fig.suptitle('Results of the Joints', fontsize = 35, fontweight = 'bold', color = 'peru')
        # Add subplots to the figure
        for joint in range(0, self.nJoints):
            self.axisPos[joint] = fig.add_subplot(4, 2, (joint + 1))
            self.axisVel[joint] = self.axisPos[joint].twinx()

        # Plot the joints position and velocity
        self.plotData()
        # Window settings
        graph = FigureCanvasTkAgg(fig, master = jointsWindow)
        graph.get_tk_widget().pack(side = "top", fill = 'both', expand = True)


    # ************************************************ #
    #                FUNCTION: saveData                #
    # ************************************************ #
    def saveData(self):
        # Files names
        filePos = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_pos.text'
        fileVel = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_vel.text'
        filesNames = [filePos, fileVel]
        # Variables to plot (time, position and velocity)
        self.t = []
        self.pos = [[] for i in range(self.nJoints)]
        self.vel = [[] for i in range(self.nJoints)]

        for file in filesNames:
            with open(file) as f:
                # Read each line of the file
                for line in f:
                    # Ignore the first two lines
                    if not line.strip().startswith("#"):
                        # Get the joints values without commas
                        jointsValues = [elt.strip() for elt in line.split(',')]

                        if file == filePos:
                            # Get and save the time
                            self.t.append(float(jointsValues[0]))
                            for value, position in zip(jointsValues[1:8], self.pos):
                                # Get and save the joints position
                                position.append((float(value) * 180) / math.pi)
                        else:
                            for value, velocity in zip(jointsValues[1:8], self.vel):
                                # Get and sabe the joints velocity
                                velocity.append((float(value) * 180) / math.pi)


    # ************************************************ #
    #                FUNCTION: plotData                #
    # ************************************************ #
    def plotData(self):
        for jointPos, jointVel, axisPos, axisVel, joint in zip(self.pos, self.vel, self.axisPos, self.axisVel, range(0, 7)):
            # Plot settings
            axisVel.yaxis.tick_left()
            axisVel.yaxis.set_label_position('left')
            axisVel.spines['left'].set_position(('outward', 45))
            axisVel.set_frame_on(True)
            axisVel.patch.set_visible(False)
            # Plot the joints position
            axisPos.set_title(('Joint ' + str(joint)), fontweight = 'bold')
            axisPos.set_xlabel('Time [s]', fontsize = 11)
            axisPos.set_xlim(min(self.t), max(self.t))
            axisPos.set_ylim([min(jointPos) - 5, max(jointPos) + 5])
            axisPos.spines['left'].set_color('firebrick')
            axisPos.spines['left'].set_linewidth(2)
            p = axisPos.plot(self.t, jointPos, color = 'firebrick', linewidth = 1.65, label = 'Pos [deg]')
            # Plot the joints velocity
            axisVel.set_ylim([min(jointVel) - 5, max(jointVel) + 5])
            axisVel.spines['left'].set_color('teal')
            axisVel.spines['left'].set_linewidth(2)
            v = axisVel.plot(self.t, jointVel, color = 'teal', linewidth = 1.65, label = 'Vel [deg/s]')
            # Legend
            lns = p + v
            labs = [l.get_label() for l in lns]
            leg = axisPos.legend(lns, labs, loc = 'lower right', fontsize = 9.5, shadow = True, fancybox = True)
            leg.get_frame().set_alpha(0.5)



# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
#                                                                                                                                    #
#                                                       Class PlotHandResults                                                        #
#                                                                                                                                    #
# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
class PlotHandResults:
    # ************************************************ #
    #                  FUNCTION: init                  #
    # ************************************************ #
    def __init__(self, root):
        # Configuration variables
        self.nJoints = 7
        self.dH_a = [0, 81, 0, 0, 0, 0, 0]
        self.dH_alpha = [0, -1.5707963268, -1.5707963268, 1.5707963268, -1.5707963268, 1.5707963268, -1.5707963268]
        self.dH_d = [317, 192.5, 400, -168.5, 400, 136.3, 265.2]
        self.TWorld = self.transMatrixInd(-0.00033700562527, -0.00000011784288577, 3.1415926536, [49.988, 14.982, 910.0])

        # Create files to store the hand position and velocity over time
        self.configFiles()
        # Get the hand position over time
        self.getHandPos()
        # Get the hand velocity over time
        self.getHandVel()

        # Figure settings
        self.handWindow = tk.Toplevel(root)
        self.fig = Figure(figsize = (15, 10), facecolor = 'gainsboro')
        # Window settings
        self.graph = FigureCanvasTkAgg(self.fig, master = self.handWindow)
        self.graph.get_tk_widget().pack(side = "top", fill = 'both', expand = True)

        # Figure title
        self.fig.suptitle('Results of the Hand', fontsize = 35, fontweight = 'bold', color = 'peru')
        # Add subplots to the figure
        gs = gridspec.GridSpec(3,1)
        self.axisPos = self.fig.add_subplot(gs[0:2], projection = '3d')
        self.axisVel = self.fig.add_subplot(gs[-1])

        # Initial view
        self.axisPos.view_init(5, 25)
        # Enable mouse click
        self.axisPos.mouse_init()

        # Plot the hand position and velocity
        self.plotData()


    # ************************************************ #
    #               FUNCTION: configFiles              #
    # ************************************************ #
    def configFiles(self):
        # Files names
        self.fileHandPos = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Hand/hand_pos.text'
        self.fileHandVel = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Hand/hand_vel.text'
        filesNames = [self.fileHandPos, self.fileHandVel]

        for file in filesNames:
             # If the file doesn't exist, it is created
            if not os.path.exists(os.path.dirname(file)):
                try:
                    os.makedirs(os.path.dirname(file))
                except OSError as exc:
                    if exc.errno != errno.EEXIST:
                        raise

            # Open the file
            with open(file, 'w') as f:
                ## In the position file: we write the file name and the variables with the units measure
                # (hand position - x, y and z in mm)
                if file == self.fileHandPos:
                    f.write('# HAND POSITION \n')
                    f.write('# x [mm], y [mm], z [mm]\n')
                # In the velocity file: we write the file name and the variables with the units measure
                # (Time in seconds and Hand Velocity in mm por seconds)#
                else:
                    f.write('# HAND VELOCITY NORM \n')
                    f.write('# Time [s], Velocity [mm/s]\n')
                # Close the file
                f.close()


    # ************************************************ #
    #               FUNCTION: getHandPos               #
    # ************************************************ #
    def getHandPos(self):
        # Variables to plot: hand position -> x, y and z
        self.handPos = [[] for i in range(0, 3)]
        # File name to open to read
        fileJointsPos = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_pos.text'
        # Open file to store the hand position
        fPos = open(self.fileHandPos, 'a')

        # Open file to read
        with open(fileJointsPos, 'r') as f:
            # Read each line of the file
            for line in f:
                # Ignore the first two lines
                if not line.strip().startswith("#"):
                    # Get the joints values without commas
                    self.jointsPosition = [elt.strip() for elt in line.split(',')]
                    # World transformation matrix
                    T = self.TWorld

                    # Determine the transformation matrix of the robot hand relative to the world
                    for a, alpha, d, theta, joint in zip(self.dH_a, self.dH_alpha, self.dH_d, self.jointsPosition[1:8], range(0, 7)):
                        if joint == 1:
                            # In the theta_1 joints: adds the offset
                            T_aux = self.transfMatix(a, alpha, d, float(theta) - 1.5707963268)
                        else:
                            T_aux = self.transfMatix(a, alpha, d, float(theta))

                        T = [[sum(a * b for a, b in zip(T_row, T_aux_col)) for T_aux_col in zip(*T_aux)] for T_row in T]

                    for i, position in zip(range(0, 3), self.handPos):
                        # Get the hand position
                        position.append(T[i][3])
                        if i == 2:
                            # Save the hand position in x and y in the file
                            fPos.write('{}\n'.format(T[i][3]))
                        else:
                            # Save the hand position in z in the file
                            fPos.write('{}, '.format(T[i][3]))


    # ************************************************ #
    #               FUNCTION: getHandPos               #
    # ************************************************ #
    def getHandVel(self):
        # Configuration variables
        jacobian = np.zeros((6,7));
        z = [0] * 3
        pos = [0] * 3
        diff = [0] * 3
        # Variables to plot: the time and the hand velocity
        self.handVel = []
        self.t = []

        # Files names to open to read
        fileJointsPos = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_pos.text'
        fileJointsVel = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_vel.text'
        # Open file to store the hand velocity
        fVel = open(self.fileHandVel, 'a')

        # Open files to read
        with open(fileJointsPos, 'r') as fJointPos, open(fileJointsVel, 'r') as fJointVel, open(self.fileHandPos, 'r') as fHandPos:
            for posJoint, velJoint, posHand in zip(fJointPos, fJointVel, fHandPos):
                # Ignore the first two lines
                if not posJoint.strip().startswith("#") and not velJoint.strip().startswith("#") and not posHand.strip().startswith("#"):
                    # Get the joints values (positon and velocity) and hand position without commas
                    self.jointsPosition = [elt.strip() for elt in posJoint.split(',')]
                    handPos = [elt.strip() for elt in posHand.split(',')]
                    jointsVelocity = [elt.strip() for elt in velJoint.split(',')]
                    # Get the time
                    time = float(jointsVelocity[0])
                    jointsVelocity = np.asarray(jointsVelocity[1:8], dtype = np.float64, order = 'C')
                    np.place(jointsVelocity, jointsVelocity == -0.001, 0.00)
                    # World transformation matrix
                    T = self.TWorld

                    for a, alpha, d, theta, joint in zip(self.dH_a, self.dH_alpha, self.dH_d, self.jointsPosition[1:8], range(0, 7)):
                        if joint == 1:
                            # In the theta_1 joints: adds the offset
                            T_aux = self.transfMatix(a, alpha, d, float(theta) - 1.5707963268)
                        else:
                            T_aux = self.transfMatix(a, alpha, d, float(theta))

                        # Determine the transformation matrix of each robot joint relative to the world
                        T = [[sum(a * b for a, b in zip(T_row, T_aux_col)) for T_aux_col in zip(*T_aux)] for T_row in T]

                        for row, col, posHand in zip(range(0, 3), T, handPos):
                            if row != 3:
                                # Z-axis orientation relative to the world
                                z[row] = col[2]
                                # Translation relative to the world
                                pos[row] = col[3]
                                # Diference between the hand position and the joint position
                                diff[row] = float(posHand) - pos[row]

                        #  Cross product
                        cross = np.cross(z, diff)
                        column = np.concatenate((cross, z), axis = None)

                        for row, col in zip(jacobian, column):
                            # Determines each column of the jacobian matrix -> each column corresponds to a joint
                            row[joint] = col

                    # Get the hand velocity norm
                    handVel = np.tensordot(jacobian, jointsVelocity, axes = 1)
                    handVelLinear = [handVel[0], handVel[1], handVel[2]]
                    handVelLinearNorm = LA.norm(handVelLinear)

                    # Get the time and the hand velocity
                    self.t.append(time)
                    self.handVel.append(handVelLinearNorm)
                    # Save the time and the hand velocity in the file
                    fVel.write('{}, {}\n'.format(time, handVelLinearNorm))


    # ************************************************ #
    #                FUNCTION: plotData                #
    # ************************************************ #
    def plotData(self):
        # Plot the hand position
        self.axisPos.set_title('Hand Position', fontweight = 'bold', fontsize = 16)
        self.axisPos.set_xlabel('x [mm]', fontsize = 10)
        self.axisPos.set_ylabel('y [mm]', fontsize = 10)
        self.axisPos.set_zlabel('z [mm]', fontsize = 10)
        self.axisPos.set_xlim([max(self.handPos[0]) + 5, min(self.handPos[0]) - 5])
        self.axisPos.set_ylim([max(self.handPos[1]) + 5, min(self.handPos[1]) - 5])
        self.axisPos.set_zlim([min(self.handPos[2]) - 5, max(self.handPos[2]) + 5])

        for i in range(0, len(self.handPos[0])):
            if i == 0:
                # Plot the start position
                self.axisPos.scatter(self.handPos[0][i], self.handPos[1][i], self.handPos[2][i], color = 'yellowgreen',
                                marker = 'o', s = 200, alpha = 0.75)
                x2D, y2D, _ = proj3d.proj_transform(self.handPos[0][i], self.handPos[1][i], self.handPos[2][i], self.axisPos.get_proj())
                self.annStart = self.axisPos.annotate('Start', xy = (x2D, y2D), xytext = (-30, 0),
                                      textcoords = 'offset points', ha = 'right', va = 'bottom',
                                      bbox = dict(boxstyle = 'round, pad = 0.5', fc = 'yellowgreen', alpha = 0.65),
                                      arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3, rad = 0'))
            elif i == len(self.handPos[0]) - 1:
                # Plot the target position
                self.axisPos.scatter(self.handPos[0][i], self.handPos[1][i], self.handPos[2][i], color = 'gold',
                                marker = 'o', s = 200, alpha = 0.75, label = 'Target')
                x2D, y2D, _ = proj3d.proj_transform(self.handPos[0][i], self.handPos[1][i], self.handPos[2][i], self.axisPos.get_proj())
                self.annTarget = self.axisPos.annotate('Target', xy = (x2D, y2D), xytext = (80, 0),
                                       textcoords = 'offset points', ha = 'right', va = 'bottom',
                                       bbox = dict(boxstyle = 'round, pad = 0.5', fc = 'gold', alpha = 0.65),
                                       arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3, rad = 0'))
            else:
                # Plot the remaining trajectory
                self.axisPos.scatter(self.handPos[0][i], self.handPos[1][i], self.handPos[2][i], color = 'firebrick',
                                marker = '.', s = 170, alpha = 0.50)

        # Plot the hand velocity
        self.axisVel.set_title('Hand Velocity Norm', fontweight = 'bold', fontsize = 16)
        self.axisVel.set_xlabel('Time [s]', fontsize = 11)
        self.axisVel.set_ylabel('[mm/s]', fontsize = 11)
        self.axisVel.set_xlim(min(self.t), max(self.t))
        self.axisVel.set_ylim(0.0, max(self.handVel) + 5)
        self.axisVel.spines['left'].set_color('teal')
        self.axisVel.spines['left'].set_linewidth(3)
        self.axisVel.plot(self.t, self.handVel, color = 'teal', linewidth = 1.65, label = 'Vel [mm/s]')


        # ************************** #
        #       FUNCTION: update     #
        # ************************** #
        def update(i):
            # Plot the start position annotation
            oldAnnStart = self.annStart
            x2D, y2D, _ = proj3d.proj_transform(self.handPos[0][0], self.handPos[1][0], self.handPos[2][0], self.axisPos.get_proj())
            self.annStart = self.axisPos.annotate('Start', xy = (x2D, y2D), xytext = (-30, 0),
                                                   textcoords = 'offset points', ha = 'right', va = 'bottom',
                                                   bbox = dict(boxstyle = 'round, pad = 0.5', fc = 'yellowgreen', alpha = 0.65),
                                                   arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3, rad = 0'))
            oldAnnStart.remove()

            # Plot the target position annotation
            oldAnnTarget = self.annTarget
            l = len(self.handPos[0]) - 1
            x2D, y2D, _ = proj3d.proj_transform(self.handPos[0][l], self.handPos[1][l], self.handPos[2][l], self.axisPos.get_proj())
            self.annTarget = self.axisPos.annotate('Target', xy = (x2D, y2D), xytext = (80, 0),
                                                    textcoords = 'offset points', ha = 'right', va = 'bottom',
                                                    bbox = dict(boxstyle = 'round, pad = 0.5', fc = 'gold', alpha = 0.65),
                                                    arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3, rad = 0'))
            oldAnnTarget.remove()

        # Updates the annotations of start and target positions, when the view is changed
        a = anim.FuncAnimation(self.fig, update, frames = 1000)
        self.handWindow.mainloop()


    # ************************************************ #
    #               FUNCTION: transfMatix              #
    # ************************************************ #
    def transfMatix(self, a, alpha, d, theta):
        # Formula of the transformation matrix with: the denavit-hartenberg parameters
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


    # ************************************************ #
    #              FUNCTION: trasnMatrixInd            #
    # ************************************************ #
    def transMatrixInd(self, alpha, beta, gama, Pos):
        # Formula of the transformation matrix with: yaw, pitch and roll rotation
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


    # ************************************************ #
    #                FUNCTION: rpyMatrix               #
    # ************************************************ #
    def rpyMatrix(self, alpha, beta, gama):
        # Formula of the rotation matrix
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



# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
#                                                                                                                                    #
#                                                         Class Application                                                          #
#                                                                                                                                    #
# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
class Application:
    # ************************************************ #
    #                  FUNCTION: init                  #
    # ************************************************ #
    def __init__(self):
        # Configuration variables
        self.start = 0
        self.stop = 0
        self.clear = 0
        self.run = 0
        # Instance an object of the JointsStates class (it is used to read the robot's joints)
        self.jointStates = JointStates()
        # Launch the application interface
        self.interface()


    # ************************************************ #
    #                FUNCTION: interface               #
    # ************************************************ #
    def interface(self):
        # Window settings
        self.root = tk.Tk()
        self.root.title("[Sawyer Robot]: Monitoring of the Movements")
        self.root.geometry("550x380")
        self.root.configure(background = 'gainsboro')

        # Font settings
        fontTitle = tkFont.Font(family = 'Helvetica', size = 12, weight = 'bold')
        self.fontButton = tkFont.Font(family = 'system', size = 10)
        self.fontStatus = tkFont.Font(family = 'system', size = 9)
        self.root.option_add('*Dialog.msg.font', 'system 10')

        # ---------------> Frame settings for the recording buttons <---------------
        recordFrame = tk.Frame(self.root, width = 50, height = 50, background = 'white')
        recordLabel = tk.Label(recordFrame, text = "Record the movement to be executed", font = fontTitle, background = 'white')
        recordLabel.pack(pady = 10)
        # Start Button
        buttonStart = tk.Button(recordFrame, text = 'Start', command = self.bStart)
        buttonStart.config(height = 2, width = 12, bg = 'indianred', font = self.fontButton)
        buttonStart.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')
        # Stop Button
        buttonStop = tk.Button(recordFrame, text = 'Stop', command = self.bStop)
        buttonStop.config(height = 2, width = 12, bg = 'lightslategray',  font = self.fontButton)
        buttonStop.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')
        # Clear Button
        buttonClear = tk.Button(recordFrame, text = 'Clear', command = self.bClear)
        buttonClear.config(height = 2, width = 12, bg = 'peru', font = self.fontButton)
        buttonClear.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')

        # ---------------> Frame settings for the results buttons <---------------
        resultsFrame = tk.Frame(self.root, width = 50, height = 50, background = 'white')
        resultsLabel = tk.Label(resultsFrame, text = "Show the results of the:", font = fontTitle, background = 'white')
        resultsLabel.pack(pady = 10)
        # Joints Buttoncoisa
        buttonJoints = tk.Button(resultsFrame, text = 'Joints', command = self.bJoints)
        buttonJoints.config(height = 2, width = 12, bg = 'goldenrod', font = self.fontButton)
        buttonJoints.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')
        # Hand Button
        buttonHand = tk.Button(resultsFrame, text = 'Hand', command = self.bHand)
        buttonHand.config(height = 2, width = 12, bg = 'darkkhaki',  font = self.fontButton)
        buttonHand.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')

        self.status = tk.Label(self.root, text = " ")
        self.status.config(font = self.fontStatus, bg = 'white', bd = 1, relief = 'ridge', anchor = 'nw')

        # Frames settings
        recordFrame.pack(side = 'top', pady = 50, fill = 'none', expand = 'no')
        resultsFrame.pack(side = 'top', fill = 'none', expand = 'no')
        self.status.pack(side = 'bottom', fill = 'x', expand = 'no')
        self.root.mainloop()


    # ************************************************ #
    #                  FUNCTION: bStop                 #
    # ************************************************ #
    def bStop(self):
        if self.run:
            # Update the satus bar
            self.status.configure(text = 'Recording: Stop  |  Results: Available')
            # Configuration variables
            self.stop = 1
            self.clear = 0
            self.run = 0
            self.start = 0
            # Unsubscribe the callback associted with "/robot/joint_states" topic
            self.jointStates.unsubscribeTopic()


    # ************************************************ #
    #                 FUNCTION: bClean                 #
    # ************************************************ #
    def bClear(self):
        if not self.run:
            # Update the satus bar
            self.status.configure(text = 'Recording: Stop  |  Results: Unavailable')
            # Configuration variables
            self.clear = 1
            self.start = 0
            self.stop = 0
            # Clean files to store the joints position and velocity
            self.jointStates.clearFiles()
        else:
            # Show a plot error if the movement recording isn't stopped
            tkMessageBox.showerror(title = 'Upssss!!!',
                                  message = 'Cannot clean the information about the executed movement! \nMake sure the movement recording is stopped.',
                                  parent = self.root)


    # ************************************************ #
    #                 FUNCTION: bStart                 #
    # ************************************************ #
    def bStart(self):
        if not self.run and not self.stop:
            # Update the satus bar
            self.status.configure(text = 'recording the movement...')
            # Configuration variables
            self.start = 1
            self.run = 1
            self.clear = 0
            # Subscribe the "/robot/joint_states" topic
            self.jointStates.subscribeTopic(dt.now())
        elif not self.run and self.stop:
            # A question is asked if you start recording a new movement without clearing the last executed movement
            question = tkMessageBox.askquestion(title = 'Record Movement',
                                                message = 'The information about the previously executed movement will be clean.\nDo you want to contine?',
                                                parent = self.root)
            if question == 'yes':
                # Update the satus bar
                self.status.configure(text = 'recording the movement...')
                # Configuration variables
                self.start = 1
                self.run = 1
                self.clear = 0
                self.stop = 0
                # Clean files to store the joints position and velocity
                self.jointStates.clearFiles()
                # Subscribe the "/robot/joint_states" topic
                self.jointStates.subscribeTopic(dt.now())


    # ************************************************ #
    #                 FUNCTION: bJoints                #
    # ************************************************ #
    def bJoints(self):
        if self.stop and not self.clear:
            # Plot the joints results
            plotJoints = PlotJointsResults(self.root)
        elif self.start:
            # Show a plot error if the movement recording isn't stopped
            tkMessageBox.showerror(title = 'Plot Error',
                                  message = 'Cannot plot the results of the joints! \nMake sure the movement recording is stopped.',
                                  parent = self.root)
        else:
            # Show a plot error if no movement has been recorded
            tkMessageBox.showerror(title = 'Plot Error',
                                  message = 'Cannot plot the results of the joints! \nMake sure the movement executed by the robot is recorded.',
                                  parent = self.root)


    # ************************************************ #
    #                 FUNCTION: bJoints                #
    # ************************************************ #
    def bHand(self):
        if self.stop and not self.clear:
            # Plot the hand results
            plotHand = PlotHandResults(self.root)
        elif self.start:
            # Show a plot error if the movement recording isn't stopped
            tkMessageBox.showerror(title = 'Plot Error',
                                   message = 'Cannot plot the results of the hand. \nMake sure the movement recording is stopped.',
                                   parent = self.root)
        else:
            # Show a plot error if no movement has been recorded
            tkMessageBox.showerror(title = 'Plot Error',
                                   message = 'Cannot plot the results of the hand. \nMake sure the movement executed by the robot is recorded.',
                                   parent = self.root)



# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
#                                                                                                                                    #
#                                                               Main                                                                 #
#                                                                                                                                    #
# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
def main():
    # Initialize ROS, where we specify the name of our node
    rospy.init_node('monitoring_interface')
    # Instance an object of the Application class
    app = Application()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
