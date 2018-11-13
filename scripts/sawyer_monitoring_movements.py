#!/usr/bin/env python
import rospy
import os
import errno
import time
import numpy as np
import math
import Tkinter as tk
import tkMessageBox
import tkFont
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import proj3d
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from sensor_msgs.msg import JointState
from datetime import datetime as dt


# ----------------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------------- #
#                                              Class JointStates                                              #
# ----------------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------------- #
class JointStates:
    # ****************************** #
    #         FUNCTION: init         #
    # ****************************** #
    def __init__(self):
        #
        self.joints_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6']
        self.nJoints = 7
        self.run = 0
        self.stop = 0
        self.save = 0
        #
        self.configFiles()


    # *************************************** #
    #         FUNCTION: subscribeNode         #
    # *************************************** #
    def subscribeTopic(self, time):
        if not self.run:
            #
            self.sub = rospy.Subscriber("/robot/joint_states", JointState, self.callbackJoints)
            #
            self.run = 1
            self.save = 1
            self.tStart = time
            self.refTime = self.tStart
            #
            self.fPos = open(self.filePos, 'a')
            self.fVel = open(self.fileVel, 'a')


    # *************************************** #
    #         FUNCTION: subscribeNode         #
    # *************************************** #
    def unsubscribeTopic(self):
        if self.run:
            #
            self.save = 0
            self.run = 0
            #
            self.fPos.close()
            self.fVel.close()


    # ************************************ #
    #         FUNCTION: clearFiles         #
    # ************************************ #
    def clearFiles(self):
        #
        self.fPos = open(self.filePos, 'w').close()
        self.fVel = open(self.fileVel, 'w').close()
        #
        self.configFiles()


    # **************************************** #
    #         FUNCTION: callbackJoints         #
    # **************************************** #
    def callbackJoints(self, data):
        pos = []
        vel = []

        #
        elapsedTime = (dt.now() - self.refTime).total_seconds()

        #
        if self.save and elapsedTime >= 0.075:
            for allJoints_name in data.name:
                #
                if allJoints_name in self.joints_names:
                    #
                    i = data.name.index(allJoints_name)
                    #
                    pos.insert(i, data.position[i])
                    #
                    vel.insert(i, data.velocity[i])

            if len(pos) == self.nJoints and len(vel) == self.nJoints:
                #
                t = (dt.now() - self.tStart).total_seconds()
                self.refTime = dt.now()
                #
                self.fPos.write('{}, {} \n'.format(t, str(pos)[1:-1]))
                self.fVel.write('{}, {} \n'.format(t, str(vel)[1:-1]))


    # ************************************* #
    #         FUNCTION: configFiles         #
    # ************************************* #
    def configFiles(self):
        #
        self.filePos = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_pos.text'
        self.fileVel = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_vel.text'
        filesNames = [self.filePos, self.fileVel]

        #
        for file in filesNames:
            #
            if not os.path.exists(os.path.dirname(file)):
                try:
                    #
                    os.makedirs(os.path.dirname(file))
                except OSError as exc:
                    #
                    if exc.errno != errno.EEXIST:
                        raise

            #
            with open(file, 'w') as f:
                #
                if file == self.filePos:
                    f.write('# JOINTS POSITION \n')
                    f.write('# Time [s], J0 [rad], J1 [rad], J2 [rad], J3 [rad], J4 [rad], J5 [rad], J6 [rad]\n')
                #
                else:
                    f.write('# JOINTS VELOCITY \n')
                    f.write('# Time [s], J0 [rad/s], J1 [rad/s], J2 [rad/s], J3 [rad/s], J4 [rad/s], J5 [rad/s], J6 [rad/s]\n')
                #
                f.close()




# ----------------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------------- #
#                                           Class PlotJointsResults                                           #
# ----------------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------------- #
class PlotJointsResults:
    # ****************************** #
    #         FUNCTION: init         #
    # ****************************** #
    def __init__(self, root):
        #
        self.nJoints = 7
        self.axisPos = [[] for i in range(self.nJoints)]
        self.axisVel = [[] for i in range(self.nJoints)]
        #
        self.saveData()

        #
        jointsWindow = tk.Toplevel(root)
        fig = Figure(figsize = (15, 12), facecolor = 'gainsboro')
        fig.subplots_adjust(top = 0.89, hspace = 0.55, wspace = 0.30)

        #
        fig.suptitle('Results of the Joints', fontsize = 35, fontweight = 'bold', color = 'peru')
        #
        for joint in range(0, self.nJoints):
            self.axisPos[joint] = fig.add_subplot(4, 2, (joint + 1))
            self.axisVel[joint] = self.axisPos[joint].twinx()

        #
        self.plotData()
        #
        graph = FigureCanvasTkAgg(fig, master = jointsWindow)
        graph.get_tk_widget().pack(side = "top", fill = 'both', expand = True)


    # ********************************** #
    #         FUNCTION: saveData         #
    # ********************************** #
    def saveData(self):
        #
        filePos = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_pos.text'
        fileVel = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_vel.text'
        filesNames = [filePos, fileVel]
        #
        self.t = []
        self.pos = [[] for i in range(self.nJoints)]
        self.vel = [[] for i in range(self.nJoints)]

        #
        for file in filesNames:
            with open(file) as f:
                #
                for line in f:
                    if not line.strip().startswith("#"):
                        #
                        jointsValues = [elt.strip() for elt in line.split(',')]

                        if file == filePos:
                            #
                            self.t.append(float(jointsValues[0]))
                            #
                            for value, position in zip(jointsValues[1:8], self.pos):
                                position.append((float(value) * 180) / math.pi)
                        else:
                            #
                            for value, velocity in zip(jointsValues[1:8], self.vel):
                                velocity.append((float(value) * 180) / math.pi)


    # ********************************** #
    #         FUNCTION: plotData         #
    # ********************************** #
    def plotData(self):
        for jointPos, jointVel, axisPos, axisVel, joint in zip(self.pos, self.vel, self.axisPos, self.axisVel, range(0, 7)):
            #
            axisVel.yaxis.tick_left()
            axisVel.yaxis.set_label_position('left')
            axisVel.spines['left'].set_position(('outward', 45))
            axisVel.set_frame_on(True)
            axisVel.patch.set_visible(False)
            #
            axisPos.set_title(('Joint ' + str(joint)), fontweight = 'bold')
            axisPos.set_xlabel('Time [s]', fontsize = 11)
            axisPos.set_xlim(min(self.t), max(self.t))
            axisPos.set_ylim([min(jointPos) - 5, max(jointPos) + 5])
            axisPos.spines['left'].set_color('firebrick')
            axisPos.spines['left'].set_linewidth(2)
            p = axisPos.plot(self.t, jointPos, color = 'firebrick', linewidth = 1.65, label = 'Pos [deg]')
            #
            axisVel.set_ylim([min(jointVel) - 5, max(jointVel) + 5])
            axisVel.spines['left'].set_color('teal')
            axisVel.spines['left'].set_linewidth(2)
            v = axisVel.plot(self.t, jointVel, color = 'teal', linewidth = 1.65, label = 'Vel [deg/s]')
            #
            lns = p + v
            labs = [l.get_label() for l in lns]
            leg = axisPos.legend(lns, labs, loc = 'lower right', fontsize = 9.5, shadow = True, fancybox = True)
            leg.get_frame().set_alpha(0.5)




# ----------------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------------- #
#                                            Class PlotHandResults                                            #
# ----------------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------------- #
class PlotHandResults:
    # ****************************** #
    #         FUNCTION: init         #
    # ****************************** #
    def __init__(self, root):
        #
        self.nJoints = 7
        self.dH_a = [0, 81, 0, 0, 0, 0, 0]
        self.dH_alpha = [0, -1.5707963268, -1.5707963268, 1.5707963268, -1.5707963268, 1.5707963268, -1.5707963268]
        self.dH_d = [317, 192.5, 400, -168.5, 400, 136.3, 265.2]
        self.TWorld = self.transMatrixInd(-0.00033700562527, -0.00000011784288577, 3.1415926536, [49.988, 14.982, 910.0])

        #
        self.configFiles()
        self.getHandPos()

        #
        handWindow = tk.Toplevel(root)
        fig = Figure(figsize = (15, 12), facecolor = 'gainsboro')

        #
        fig.suptitle('Results of the Hand', fontsize = 35, fontweight = 'bold', color = 'peru')
        #
        gs = gridspec.GridSpec(3,1)
        self.axisPos = fig.add_subplot(gs[0:2], projection = '3d')
        self.axisPos.view_init(5, 25)
        self.axisVel = fig.add_subplot(gs[-1])

        #
        self.plotData()
        #
        graph = FigureCanvasTkAgg(fig, master = handWindow)
        self.axisPos.mouse_init()
        graph.get_tk_widget().pack(side = "top", fill = 'both', expand = True)


    # ************************************* #
    #         FUNCTION: configFiles         #
    # ************************************* #
    def configFiles(self):
        self.fileHandPos = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Hand/hand_pos.text'
        self.fileHandVel = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Hand/hand_vel.text'
        filesNames = [self.fileHandPos, self.fileHandVel]

        #
        for file in filesNames:
            #
            if not os.path.exists(os.path.dirname(file)):
                try:
                    #
                    os.makedirs(os.path.dirname(file))
                except OSError as exc:
                    #
                    if exc.errno != errno.EEXIST:
                        raise

            #
            with open(file, 'w') as f:
                #
                if file == self.fileHandPos:
                    f.write('# HAND POSITION \n')
                    f.write('# x [mm], y [mm], z [mm]\n')
                #
                else:
                    f.write('# HAND VELOCITY \n')
                    f.write('# x [mm/s], y [mm/s], z [mm/s]\n')
                f.close()


    # ************************************ #
    #         FUNCTION: getHandPos         #
    # ************************************ #
    def getHandPos(self):
        #
        fileJointsPos = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_pos.text'
        self.handPos = [[] for i in range(0, 3)]

        self.fPos = open(self.fileHandPos, 'a')

        with open(fileJointsPos) as f:
            for line in f:
                if not line.strip().startswith("#"):
                    #
                    jointsValues = [elt.strip() for elt in line.split(',')]
                    T = self.TWorld

                    for a, alpha, d, theta, joint in zip(self.dH_a, self.dH_alpha, self.dH_d, jointsValues[1:8], range(0, 7)):
                        if joint == 1:
                            #
                            T_aux = self.transfMatix(a, alpha, d, float(theta) - 1.5707963268)
                        else:
                            #
                            T_aux = self.transfMatix(a, alpha, d, float(theta))
                        #
                        T = [[sum(a * b for a, b in zip(T_row, T_aux_col)) for T_aux_col in zip(*T_aux)] for T_row in T]

                    for i, position in zip(range(0, 3), self.handPos):
                        #
                        position.append(T[i][3])
                        if i == 2:
                            self.fPos.write('{}\n'.format(T[i][3]))
                        else:
                            self.fPos.write('{}, '.format(T[i][3]))


    # ********************************** #
    #         FUNCTION: plotData         #
    # ********************************** #
    def plotData(self):
        self.axisPos.set_title('Hand Position', fontweight = 'bold', fontsize = 18)
        self.axisPos.set_xlabel('y [mm]', fontsize = 10)
        self.axisPos.set_ylabel('x [mm]', fontsize = 10)
        self.axisPos.set_zlabel('z [mm]', fontsize = 10)
        self.axisPos.set_xlim([min(self.handPos[1]) - 1, max(self.handPos[1]) + 1])
        self.axisPos.set_ylim([min(self.handPos[0]) + 1, max(self.handPos[0]) - 1])
        self.axisPos.set_zlim([min(self.handPos[2]) - 1, max(self.handPos[2]) + 1])

        for i in range(0, len(self.handPos[0])):
            if i == 0:
                self.axisPos.scatter(self.handPos[1][i], self.handPos[0][i], self.handPos[2][i], color = 'cornflowerblue',
                                marker = '^', s = 175, alpha = 1.0)
                x2D, y2D, _ = proj3d.proj_transform(self.handPos[1][i], self.handPos[0][i], self.handPos[2][i], self.axisPos.get_proj())
                self.axisPos.annotate('Start', xy = (x2D, y2D), xytext = (-30, 0),
                                 textcoords = 'offset points', ha = 'right', va = 'bottom',
                                 bbox = dict(boxstyle = 'round, pad = 0.5', fc = 'cornflowerblue', alpha = 0.45),
                                 arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3, rad = 0'))
            elif i == len(self.handPos[0]) - 1:
                self.axisPos.scatter(self.handPos[1][i], self.handPos[0][i], self.handPos[2][i], color = 'forestgreen',
                                marker = 'v', s = 175, alpha = 1.0)
                x2D, y2D, _ = proj3d.proj_transform(self.handPos[1][i], self.handPos[0][i], self.handPos[2][i], self.axisPos.get_proj())
                self.axisPos.annotate('Current', xy = (x2D, y2D), xytext = (80, 0),
                                 textcoords = 'offset points', ha = 'right', va = 'bottom',
                                 bbox = dict(boxstyle = 'round, pad = 0.5', fc = 'forestgreen', alpha = 0.45),
                                 arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3, rad = 0'))
            else:
                self.axisPos.scatter(self.handPos[1][i], self.handPos[0][i], self.handPos[2][i], color = 'firebrick',
                                marker = '.', s = 170, alpha = 0.50)


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




# ----------------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------------- #
#                                              Class Application                                              #
# ----------------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------------- #
class Application:
    # ****************************** #
    #         FUNCTION: init         #
    # ****************************** #
    def __init__(self):
        #
        self.start = 0
        self.stop = 0
        self.clear = 0
        self.run = 0
        #
        self.jointStates = JointStates()
        self.interface()


    # *********************************** #
    #         FUNCTION: interface         #
    # *********************************** #
    def interface(self):
        #
        self.root = tk.Tk()
        self.root.title("[Sawyer Robot]: Monitoring of the Movements")
        self.root.geometry("550x375")
        self.root.configure(background = 'gainsboro')

        #
        fontTitle = tkFont.Font(family = 'Helvetica', size = 12, weight = 'bold')
        self.fontButton = tkFont.Font(family = 'system', size = 10)
        self.root.option_add('*Dialog.msg.font', 'system 10')

        #***************************
        recordFrame = tk.Frame(self.root, width = 50, height = 50, background = 'white')
        recordLabel = tk.Label(recordFrame, text = "Record the movement to be executed", font = fontTitle, background = 'white')
        recordLabel.pack(pady = 10)
        #
        buttonStart = tk.Button(recordFrame, text = 'Start', command = self.bStart)
        buttonStart.config(height = 2, width = 12, bg = 'indianred', font = self.fontButton)
        buttonStart.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')
        #
        buttonStop = tk.Button(recordFrame, text = 'Stop', command = self.bStop)
        buttonStop.config(height = 2, width = 12, bg = 'lightslategray',  font = self.fontButton)
        buttonStop.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')
        #
        buttonClear = tk.Button(recordFrame, text = 'Clear', command = self.bClear)
        buttonClear.config(height = 2, width = 12, bg = 'peru', font = self.fontButton)
        buttonClear.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')

        #***************************
        resultsFrame = tk.Frame(self.root, width = 50, height = 50, background = 'white')
        resultsLabel = tk.Label(resultsFrame, text = "Show the results of the:", font = fontTitle, background = 'white')
        resultsLabel.pack(pady = 10)
        #
        buttonJoints = tk.Button(resultsFrame, text = 'Joints', command = self.bJoints)
        buttonJoints.config(height = 2, width = 12, bg = 'goldenrod', font = self.fontButton)
        buttonJoints.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')
        #
        buttonHand = tk.Button(resultsFrame, text = 'Hand', command = self.bHand)
        buttonHand.config(height = 2, width = 12, bg = 'darkkhaki',  font = self.fontButton)
        buttonHand.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')

        #***************************
        recordFrame.pack(side = 'top', pady = 50, fill = 'none', expand = 'no')
        resultsFrame.pack(side = 'top', fill = 'none', expand = 'no')
        self.root.mainloop()


    # ******************************* #
    #         FUNCTION: bStop         #
    # ******************************* #
    def bStop(self):
        if self.run:
            self.stop = 1
            self.clear = 0
            self.run = 0
            self.start = 0
            self.jointStates.unsubscribeTopic()


    # ******************************** #
    #         FUNCTION: bClean         #
    # ******************************** #
    def bClear(self):
        if not self.run:
            self.clear = 1
            self.start = 0
            self.stop = 0
            self.jointStates.clearFiles()


    # ******************************** #
    #         FUNCTION: bStart         #
    # ******************************** #
    def bStart(self):
        if not self.run and not self.stop:
            self.start = 1
            self.run = 1
            self.clear = 0
            self.jointStates.subscribeTopic(dt.now())
        elif not self.run and self.stop:
            question = tkMessageBox.askquestion(title = 'Record Movement',
                                                message = 'The information about the previously executed movement will be clean.\nDo you want to contine?',
                                                parent = self.root)
            if question == 'yes':
                self.start = 1
                self.run = 1
                self.clear = 0
                self.stop = 0
                self.jointStates.clearFiles()
                self.jointStates.subscribeTopic(dt.now())


    # ********************************* #
    #         FUNCTION: bJoints         #
    # ********************************* #
    def bJoints(self):
        if self.stop and not self.clear:
            plotJoints = PlotJointsResults(self.root)
        elif self.start:
            tkMessageBox.showerror(title = 'Plot Error',
                                  message = 'Cannot plot the results of the joints! \nMake sure the movement recording is stopped.',
                                  parent = self.root)
        else:
            tkMessageBox.showerror(title = 'Plot Error',
                                  message = 'Cannot plot the results of the joints! \nMake sure the movement executed by the robot is recorded.',
                                  parent = self.root)


    # ********************************* #
    #         FUNCTION: bJoints         #
    # ********************************* #
    def bHand(self):
        if self.stop and not self.clear:
            plotHand = PlotHandResults(self.root)
        elif self.start:
            tkMessageBox.showerror(title = 'Plot Error',
                                   message = 'Cannot plot the results of the hand. \nMake sure the movement recording is stopped.',
                                   parent = self.root)
        else:
            tkMessageBox.showerror(title = 'Plot Error',
                                   message = 'Cannot plot the results of the hand. \nMake sure the movement executed by the robot is recorded.',
                                   parent = self.root)




# ----------------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------------- #
#                                                    Main                                                     #
# ----------------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------------- #
def main():
    rospy.init_node('monitoring_interface')
    app = Application()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
