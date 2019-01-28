#!/usr/bin/env python
import os
import errno
import numpy as np
import math
import scipy.signal
import mpl_toolkits.axes_grid1
import matplotlib.pyplot as plt

class SavitzkyGolay:
    # ************************************************ #
    #                    FUNCTION: init                #
    # ************************************************ #
    def __init__(self):
        self.fileData = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/Joints/joints_vel_noise.text'
        self.nJoints = 7

        self.configFiles()
        self.getData()

        self.filterData(2, 15)
        self.saveFilterData(self.fileVel_o2_w13)
        self.filterData(3, 15)
        self.saveFilterData(self.fileVel_o3_w13)
        self.filterData(4, 15)
        self.saveFilterData(self.fileVel_o4_w13)
        self.filterData(2, 11)
        self.saveFilterData(self.fileVel_o2_w101)
        self.filterData(3, 11)
        self.saveFilterData(self.fileVel_o3_w101)
        self.filterData(4, 11)
        self.saveFilterData(self.fileVel_o4_w101)

        self.getFilteredData()
        self.plotData()


    # ************************************************ #
    #               FUNCTION: saveFilterData           #
    # ************************************************ #
    def getFilteredData(self):
        # Variables to plot (time, position and velocity)
        self.t = []
        self.vel_o2_w13 = [[] for i in range(self.nJoints)]
        self.vel_o3_w13 = [[] for i in range(self.nJoints)]
        self.vel_o4_w13 = [[] for i in range(self.nJoints)]
        self.vel_o2_w101 = [[] for i in range(self.nJoints)]
        self.vel_o3_w101 = [[] for i in range(self.nJoints)]
        self.vel_o4_w101 = [[] for i in range(self.nJoints)]


        for file in self.filesNames:
            with open(file) as f:
                # Read each line of the file
                for line in f:
                    # Ignore the first two lines
                    if not line.strip().startswith("#"):
                        # Get the joints values without commas
                        jointsValues = [elt.strip() for elt in line.split(',')]

                        #
                        if file == self.fileVel_o2_w13:
                            self.t.append(float(jointsValues[0]))
                            for value, velocity in zip(jointsValues[1:8], self.vel_o2_w13):
                                velocity.append((float(value) * 180) / math.pi)
                        #
                        elif file == self.fileVel_o3_w13:
                            for value, velocity in zip(jointsValues[1:8], self.vel_o3_w13):
                                velocity.append((float(value) * 180) / math.pi)
                        #
                        elif file == self.fileVel_o4_w13:
                            for value, velocity in zip(jointsValues[1:8], self.vel_o4_w13):
                                velocity.append((float(value) * 180) / math.pi)
                        #
                        elif file == self.fileVel_o2_w101:
                            for value, velocity in zip(jointsValues[1:8], self.vel_o2_w101):
                                velocity.append((float(value) * 180) / math.pi)
                        #
                        elif file == self.fileVel_o3_w101:
                            for value, velocity in zip(jointsValues[1:8], self.vel_o3_w101):
                                velocity.append((float(value) * 180) / math.pi)
                        #
                        elif file == self.fileVel_o4_w101:
                            for value, velocity in zip(jointsValues[1:8], self.vel_o4_w101):
                                velocity.append((float(value) * 180) / math.pi)
            #
            f.close()


    # ************************************************ #
    #               FUNCTION: saveFilterData           #
    # ************************************************ #
    def plotData(self):
        self.axisVel = [[] for i in range(self.nJoints)]

        # *-*-*-*-*-*-*-*-*-*- #
        #   Create the figure  #
        fig = plt.figure()
        fig.subplots_adjust(top = 0.89, hspace = 0.55, wspace = 0.30)
        # Maximize the figure
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())

        # Title of the figure
        fig.suptitle('[Savitzky Golay Filter]: Smooth Joints Velocities', fontsize = 35, fontweight = 'bold', color = 'peru')
        # Create subplots of the figure
        for joint in range(0, self.nJoints):
            self.axisVel[joint] = fig.add_subplot(4, 2, (joint + 1))
        #self.axisVel = fig.add_subplot(1, 1, 1)

        for jointVel_f1, jointVel_f2, jointVel_f3, jointVel_f4, jointVel_f5, jointVel_f6, original, axis, joint in zip(self.vel_o2_w13, self.vel_o3_w13, self.vel_o4_w13, self.vel_o2_w101, self.vel_o3_w101,
            self.vel_o4_w101, self.jointsVelNoiseDegrees, self.axisVel, range(0, 7)):
            axis.set_title(('Joint ' + str(joint)), fontweight = 'bold')
            axis.set_xlabel('Time [s]', fontsize = 11)
            axis.set_xlim(min(self.t), max(self.t))
            axis.plot(self.t, original, linewidth = 2, color = 'darkgoldenrod', label = 'original')
            #axis.plot(self.t, jointVel_f1, linewidth = 1, color = 'indianred', label = 'O = 2, W = 13')
            #axis.plot(self.t, jointVel_f2, linewidth = 1, color = 'teal', label = 'O = 3, W = 13')
            #axis.plot(self.t, jointVel_f3, linewidth = 1, color = 'chocolate', label = 'O = 4, W = 13')
            #axis.plot(self.t, jointVel_f4, linewidth = 1, color = 'black', label = 'O = 2, W = 101')
            axis.plot(self.t, jointVel_f5, linewidth = 1, color = 'purple', label = 'O = 3, W = 13')
            #axis.plot(self.t, jointVel_f6, linewidth = 1, color = 'darkslategrey', label = 'O = 4, W = 101')
            axis.legend()

        #self.axisVel.set_title(('Joint ' + str(0)), fontweight = 'bold')
        #self.axisVel.set_xlabel('Time [s]', fontsize = 11)
        #self.axisVel.set_xlim(min(self.t), max(self.t))
        #self.axisVel.plot(self.t, self.jointsVelNoiseDegrees[0], linewidth = 2, color = 'darkgoldenrod', label = 'original')
        #self.axisVel.plot(self.t, self.vel_o2_w13[0], linewidth = 2, color = 'indianred', label = 'O = 2, W = 15')
        #self.axisVel.plot(self.t, self.vel_o3_w13[0], linewidth = 2, color = 'teal', label = 'O = 3, W = 15')
        #self.axisVel.plot(self.t, self.vel_o4_w13[0], linewidth = 2, color = 'chocolate', label = 'O = 4, W = 15')
        #self.axisVel.plot(self.t, self.vel_o2_w101[0], linewidth = 2, color = 'black', label = 'O = 2, W = 13')
        #self.axisVel.plot(self.t, self.vel_o3_w101[0], linewidth = 2, color = 'purple', label = 'O = 3, W = 13')
        #self.axisVel.plot(self.t, self.vel_o4_w101[0], linewidth = 2, color = 'darkslategrey', label = 'O = 4, W = 13')
        #self.axisVel.legend()
        plt.show()


    # ************************************************ #
    #               FUNCTION: saveFilterData           #
    # ************************************************ #
    def saveFilterData(self, fileName):
        #
        file = open(fileName, 'a')

        for t, value in zip(self.t, range(0 , len(self.jointsVel[0]))):
            #
            jointsVelFilter = [joint[value] for joint in self.jointsVel]
            jointsVelFilter = [round(value, 4) for value in jointsVelFilter]
            jointsVelFilter = ', '.join(map(repr, jointsVelFilter))
            #
            file.write('{}, {}\n'.format(t, jointsVelFilter))

        #
        file.close()


    # ************************************************ #
    #                 FUNCTION: filterData             #
    # ************************************************ #
    def filterData(self, order, windowLength):
        #
        self.jointsVel = []

        for jointVel in self.jointsVelNoise:
            #
            velWithoutNoise = scipy.signal.savgol_filter(np.array(jointVel, dtype = np.float64), windowLength, order, mode = 'mirror')
            self.jointsVel.append(velWithoutNoise.tolist())


    # ************************************************ #
    #                 FUNCTION: getData                #
    # ************************************************ #
    def getData(self):
        #
        self.jointsVelNoise = [[] for i in range(self.nJoints)]
        self.jointsVelNoiseDegrees = [[] for i in range(self.nJoints)]
        self.t = []

        with open(self.fileData) as f:
            # Read each line of the file
            for line in f:
                # Ignore the first two lines
                if not line.strip().startswith("#"):
                    # Get the joints values without commas
                    jointsVelocities = [elt.strip() for elt in line.split(',')]
                    self.t.append(jointsVelocities[0])

                    for value, velocity, velDegrees in zip(jointsVelocities[1:8], self.jointsVelNoise, self.jointsVelNoiseDegrees):
                        # Get and sabe the joints velocity
                        velocity.append(float(value))
                        velDegrees.append((float(value) * 180) / math.pi)


    # ************************************************ #
    #                 FUNCTION: configFiles            #
    # ************************************************ #
    def configFiles(self):
        # Files names
        self.fileVel_o2_w13 = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/TestFilter/joints_vel_o2_w13.text'
        self.fileVel_o3_w13 = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/TestFilter/joints_vel_o3_w13.text'
        self.fileVel_o4_w13 = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/TestFilter/joints_vel_o4_w13.text'
        self.fileVel_o2_w101 = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/TestFilter/joints_vel_o2_w101.text'
        self.fileVel_o3_w101 = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/TestFilter/joints_vel_o3_w101.text'
        self.fileVel_o4_w101 = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/TestFilter/joints_vel_o4_w101.text'

        self.filesNames = [self.fileVel_o2_w13, self.fileVel_o3_w13, self.fileVel_o4_w13, self.fileVel_o2_w101, self.fileVel_o3_w101, self.fileVel_o4_w101]

        for file in self.filesNames:
            # If the file doesn't exist, it is created.
            if not os.path.exists(os.path.dirname(file)):
                try:
                    os.makedirs(os.path.dirname(file))
                except OSError as exc:
                    if exc.errno != errno.EEXIST:
                        raise

            # Open the file
            with open(file, 'w') as f:
                f.write('# SMOOTHED JOINTS VELOCITY \n')
                f.write('# Time [s], J0 [rad/s], J1 [rad/s], J2 [rad/s], J3 [rad/s], J4 [rad/s], J5 [rad/s], J6 [rad/s]\n')


# ************************************************ #
#                   FUNCTION: main                 #
# ************************************************ #
def main():
    filter = SavitzkyGolay()

if __name__ == '__main__':
    main()
