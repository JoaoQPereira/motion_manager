#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
import mpl_toolkits.axes_grid1
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.widgets import Button
from sensor_msgs.msg import JointState

# ----------------------------------------------------------------------------------------------------------- #
#                                              Class JointStates                                              #
# ----------------------------------------------------------------------------------------------------------- #
class JointStates:
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

        # Topic to subcribe to
        rospy.Subscriber("/robot/joint_states", JointState, self.callbackJoints)
        rate = rospy.Rate(10) # Hz

        while not rospy.is_shutdown():
            # Plot the data
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
        # Initialization of variables
        self.t = []
        self.pos = [[] for i in range(self.nJoints)]
        self.vel = [[] for i in range(self.nJoints)]
        self.axisPos = [[] for i in range(self.nJoints)]
        self.axisVel = [[] for i in range(self.nJoints)]

        # *-*-*-*-*-*-*-*-*-*- #
        #   Create the figure  #
        fig = plt.figure()
        fig.subplots_adjust(top = 0.89, hspace = 0.55, wspace = 0.30)
        # Maximize the figure
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())

        # *-*-*-*-*-*-*-*-*-*-* #
        #   Initialize buttons  #
        # Start
        axStart = plt.axes([0.60625, 0.11, 0.25, 0.04])
        buttonStart = Button(axStart, 'Start', color = 'snow', hovercolor = 'tan')
        buttonStart.on_clicked(self.bStart)
        # Stop
        divider = mpl_toolkits.axes_grid1.make_axes_locatable(axStart)
        axStop = divider.append_axes("right", size = "100%", pad = 0.25)
        buttonStop = Button(axStop, 'Stop', color = 'snow', hovercolor = 'tan')
        buttonStop.on_clicked(self.bStop)
        # Clear
        axClear = divider.append_axes("right", size = "100%", pad = 0.25)
        buttonClear = Button(axClear, 'Clear', color = 'snow', hovercolor = 'tan')
        buttonClear.on_clicked(self.bClear)
        # Text box
        fig.patches.extend([plt.Rectangle((0.5955,  0.1), 0.2720, 0.1, fill = True, color = 'forestgreen', alpha = 0.1, zorder = 1000,
                                           transform = fig.transFigure, figure = fig)])
        plt.text(-0.70, 1.5, 'Record the movement to be executed', fontweight = 'bold', horizontalalignment='center')

        # Title of the figure
        fig.suptitle('[Sawyer Robot]: Monitoring of Joints Movements', fontsize = 35, fontweight = 'bold', color = 'peru')
        # Create subplots of the figure
        for joint in range(0, self.nJoints):
            self.axisPos[joint] = fig.add_subplot(4, 2, (joint + 1))
            self.axisVel[joint] = self.axisPos[joint].twinx()

        # ******************* #
        #   FUNCTION: update  #
        # ******************* #
        def update(i):
            if self.joints_pos and self.StopPlot == 0:
                # *-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
                #   Initilization of variables  #
                # Joint number
                joint = 0
                # *-*-*-* #
                #   Time  #
                if self.iTime == 0:
                    self.t.append(0.0)
                    self.iTime = time.clock()
                else:
                    self.t.append(time.clock() - self.iTime)

                # *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
                #   Plot Joints Position and Velocity   #
                for jointPos, jointVel, posValue, velValue, axisPos, axisVel in zip(
                        self.pos, self.vel, self.joints_pos, self.joints_vel, self.axisPos, self.axisVel):
                    # *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* #
                    #   Joints position and velocity  #
                    jointPos.append((float(posValue) * 180) / math.pi)
                    jointVel.append((float(velValue) * 180) / math.pi)
                    # *-*-*-*-*-*-*-*- #
                    #   Plot the data  #
                    axisPos.clear()
                    # Configurations
                    axisVel.yaxis.tick_left()
                    axisVel.yaxis.set_label_position('left')
                    axisVel.spines['left'].set_position(('outward', 45))
                    axisVel.set_frame_on(True)
                    axisVel.patch.set_visible(False)
                    # Plot the position
                    axisPos.set_title(('Joint ' + str(joint)), fontweight = 'bold')
                    axisPos.set_xlabel('Time [s]', fontsize = 11)
                    axisPos.set_xlim([max(0, self.t[-1] - 40), self.t[-1] + 0.0001])
                    axisPos.set_ylim([min(jointPos) - 10, max(jointPos) + 10])
                    axisPos.spines['left'].set_color('firebrick')
                    axisPos.spines['left'].set_linewidth(2)
                    p = axisPos.plot(self.t, jointPos, color = 'firebrick', linewidth = 1.65, label = 'Pos [deg]')
                    # Plot the velocity
                    axisVel.set_ylim([min(jointVel) - 5, max(jointVel) + 5])
                    axisVel.spines['left'].set_color('teal')
                    axisVel.spines['left'].set_linewidth(2)
                    v = axisVel.plot(self.t, jointVel, color = 'teal', linewidth = 1.65, label = 'Vel [deg/s]')
                    # Legend
                    lns = p + v
                    labs = [l.get_label() for l in lns]
                    leg = axisPos.legend(lns, labs, loc = 'lower right', fontsize = 9.5, shadow = True, fancybox = True)
                    leg.get_frame().set_alpha(0.5)
                    joint = joint + 1
            else:
                pass

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
        self.t = []
        self.iTime = 0
        self.pos = [[] for i in range(0, self.nJoints)]
        self.vel = [[] for i in range(0, self.nJoints)]
        for axisPos, axisVel, valuePos, valueVel in zip(self.axisPos, self.axisVel, self.joints_pos, self.joints_vel):
            axisPos.set_xlim([0, self.iTime + 0.0001])
            axisPos.set_ylim([((float(valuePos) * 180) / math.pi) - 10, ((float(valuePos) * 180) / math.pi) + 10])
            axisVel.set_ylim([((float(valueVel) * 180) / math.pi) - 5, ((float(valueVel) * 180) / math.pi) + 5])

    # ******************* #
    #   FUNCTION: bStart  #
    # ******************* #
    def bStart(self, event):
        self.StopPlot = 0

# ----------------------------------------------------------------------------------------------------------- #
#                                                    Main                                                     #
# ----------------------------------------------------------------------------------------------------------- #
def main():
    rospy.init_node('monitoring_interface_joints')
    listener = JointStates()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
