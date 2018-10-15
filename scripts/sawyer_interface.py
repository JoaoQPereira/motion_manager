#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# ----------------------------------------------------------------------------------------------------------- #
#                                              Class JointStates                                              #
# ----------------------------------------------------------------------------------------------------------- #
class JointStates:
    # ***************** #
    #   FUNCTION: init  #
    # ***************** #
    def __init__(self):
        self.joints_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6']
        self.joints_pos = []
        self.joints_vel = []
        self.nJoints = 7

        rospy.Subscriber("/robot/joint_states", JointState, self.callbackJoints)
        rate = rospy.Rate(10) # Hz

        while not rospy.is_shutdown():
            #rospy.loginfo('name:{}, pos:{}, vel:{}'.format(self.joints_names, self.joints_pos, self.joints_vel))
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
                i = data.name.index(allJoints_name)
                pos.insert(i, data.position[i])
                vel.insert(i, data.velocity[i])

        self.joints_pos = pos
        self.joints_vel = vel

    # ********************* #
    #   FUNCTION: plotData  #
    # ********************* #
    def plotData(self):
        t = []
        pos = [[] for _ in range(self.nJoints)]
        vel = [[] for _ in range(self.nJoints)]

        fig = plt.figure()
        fig.subplots_adjust(top = 0.87, hspace = 0.3, wspace = 0.2)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())

        fig.suptitle('[Sawyer Robot]: Result of the joints', fontsize = 35, fontweight = 'bold', color = 'peru')
        axisJ0 = fig.add_subplot(2, 4, 1)
        axisJ1 = fig.add_subplot(2, 4, 2)
        axisJ2 = fig.add_subplot(2, 4, 3)
        axisJ3 = fig.add_subplot(2, 4, 4)
        axisJ4 = fig.add_subplot(2, 4, 5)
        axisJ5 = fig.add_subplot(2, 4, 6)
        axisJ6 = fig.add_subplot(2, 4, 7)

        def update(i):
            if self.joints_pos:
                t.append(time.clock())
                for x in range(0, self.nJoints):
                    pos[x].append((float(self.joints_pos[x]) * 180) / math.pi)
                    vel[x].append((float(self.joints_vel[x]) * 180) / math.pi)

                # Joint 0
                axisJ0.clear()
                axisJ0.set_xlim(left = max(0, t[-1] - 20), right = t[-1] + 20)
                axisJ0.set_title('Joint 0', fontweight = 'bold')
                axisJ0.set_xlabel('Time [s]')
                axisJ0.plot(t, pos[0], color = 'cornflowerblue', linewidth = 1.5, label = 'Pos [deg]')
                axisJ0.plot(t, vel[0], color = 'crimson', linewidth = 1.5, label = 'Vel [deg/s]')
                axisJ0.legend(('Pos [deg]', 'Vel [deg/s]'), fontsize = 11, loc='lower right', shadow = True, fancybox = True)
                # Joint 1
                axisJ1.clear()
                axisJ1.set_xlim(left = max(0, t[-1] - 20), right = t[-1] + 20)
                axisJ1.set_title('Joint 1', fontweight = 'bold')
                axisJ1.set_xlabel('Time [s]')
                axisJ1.plot(t, pos[1], color = 'cornflowerblue', linewidth = 1.5, label = 'Pos [deg]')
                axisJ1.plot(t, vel[1], color = 'crimson', linewidth = 1.5, label = 'Vel [deg/s]')
                axisJ1.legend(('Pos [deg]', 'Vel [deg/s]'), fontsize = 11, loc='lower right', shadow = True, fancybox = True)
                # Joint 2
                axisJ2.clear()
                axisJ2.set_xlim(left = max(0, t[-1] - 20), right = t[-1] + 20)
                axisJ2.set_title('Joint 2', fontweight = 'bold')
                axisJ2.set_xlabel('Time [s]')
                axisJ2.plot(t, pos[2], color = 'cornflowerblue', linewidth = 1.5, label = 'Pos [deg]')
                axisJ2.plot(t, vel[2], color = 'crimson', linewidth = 1.5, label = 'Vel [deg/s]')
                axisJ2.legend(('Pos [deg]', 'Vel [deg/s]'), fontsize = 11, loc='lower right', shadow = True, fancybox = True)
                # Joint 3
                axisJ3.clear()
                axisJ3.set_xlim(left = max(0, t[-1] - 20), right = t[-1] + 20)
                axisJ3.set_title('Joint 3', fontweight = 'bold')
                axisJ3.set_xlabel('Time [s]')
                axisJ3.plot(t, pos[3], color = 'cornflowerblue', linewidth = 1.5, label = 'Pos [deg]')
                axisJ3.plot(t, vel[3], color = 'crimson', linewidth = 1.5, label = 'Vel [deg/s]')
                axisJ3.legend(('Pos [deg]', 'Vel [deg/s]'), fontsize = 11, loc='lower right', shadow = True, fancybox = True)
                # Joint 4
                axisJ4.clear()
                axisJ4.set_xlim(left = max(0, t[-1] - 20), right = t[-1] + 20)
                axisJ4.set_title('Joint 4', fontweight = 'bold')
                axisJ4.set_xlabel('Time [s]')
                axisJ4.plot(t, pos[4], color = 'cornflowerblue', linewidth = 1.5, label = 'Pos [deg]')
                axisJ4.plot(t, vel[4], color = 'crimson', linewidth = 1.5, label = 'Vel [deg/s]')
                axisJ4.legend(('Pos [deg]', 'Vel [deg/s]'), fontsize = 11, loc='lower right', shadow = True, fancybox = True)
                # Joint 5
                axisJ5.clear()
                axisJ5.set_xlim(left = max(0, t[-1] - 20), right = t[-1] + 20)
                axisJ5.set_title('Joint 5', fontweight = 'bold')
                axisJ5.set_xlabel('Time [s]')
                axisJ5.plot(t, pos[5], color = 'cornflowerblue', linewidth = 1.5, label = 'Pos [deg]')
                axisJ5.plot(t, vel[5], color = 'crimson', linewidth = 1.5, label = 'Vel [deg/s]')
                axisJ5.legend(('Pos [deg]', 'Vel [deg/s]'), fontsize = 11, loc='lower right', shadow = True, fancybox = True)
                # Joint 6
                axisJ6.clear()
                axisJ6.set_xlim(left = max(0, t[-1] - 20), right = t[-1] + 20)
                axisJ6.set_title('Joint 6', fontweight = 'bold')
                axisJ6.set_xlabel('Time [s]')
                axisJ6.plot(t, pos[6], color = 'cornflowerblue', linewidth = 1.5, label = 'Pos [deg]')
                axisJ6.plot(t, vel[6], color = 'crimson', linewidth = 1.5, label = 'Vel [deg/s]')
                axisJ6.legend(('Pos [deg]', 'Vel [deg/s]'), fontsize = 11, loc='lower right', shadow = True, fancybox = True)
            else:
                pass

        a = anim.FuncAnimation(fig, update, frames = 100000)
        plt.show()

# ----------------------------------------------------------------------------------------------------------- #
#                                                    Main                                                     #
# ----------------------------------------------------------------------------------------------------------- #
def main():
    rospy.init_node('motion_interface')
    listener = JointStates()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
