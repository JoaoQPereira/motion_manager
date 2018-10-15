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
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())

        fig.suptitle('[Sawyer Robot]: Result of the joints', fontsize = 35, fontweight = 'bold', color = 'peru')
        fig.subplots_adjust(top = 0.87, hspace = 0.30)
        axisPos = fig.add_subplot(2, 1, 1)
        axisVel = fig.add_subplot(2, 1, 2)

        def update(i):
            if self.joints_pos:
                t.append(time.clock())
                for x in range(0, self.nJoints):
                    pos[x].append((float(self.joints_pos[x]) * 180) / math.pi)
                    vel[x].append((float(self.joints_vel[x]) * 180) / math.pi)

                # Joints Position
                axisPos.clear()
                axisPos.set_xlim(left = max(0, t[-1] - 90), right = t[-1] + 90)
                axisPos.set_title('Joints Positions', fontweight = 'bold')
                axisPos.set_xlabel('Time [s]')
                axisPos.set_ylabel('Position [deg]')
                axisPos.plot(t, pos[0], color = 'orchid', linewidth = 1.4, label = 'J0')
                axisPos.plot(t, pos[1], color = 'orange', linewidth = 1.4, label = 'J1')
                axisPos.plot(t, pos[2], color = 'cornflowerblue', linewidth = 1.4, label = 'J2')
                axisPos.plot(t, pos[3], color = 'olivedrab', linewidth = 1.4, label = 'J3')
                axisPos.plot(t, pos[4], color = 'peru', linewidth = 1.4, label = 'J4')
                axisPos.plot(t, pos[5], color = 'crimson', linewidth = 1.4, label = 'J5')
                axisPos.plot(t, pos[6], color = 'royalblue', linewidth = 1.4, label = 'J6')
                axisPos.legend(('Joint 0', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'),
                           loc='lower right', ncol = 2, shadow = True, fancybox = True)

                # Joints Velocity
                axisVel.clear()
                axisVel.set_xlim(left = max(0, t[-1] - 90), right = t[-1] + 90)
                axisVel.set_title('Joints Velocity', fontweight = 'bold')
                axisVel.set_xlabel('Time [s]')
                axisVel.set_ylabel('Velocity [deg/s]')
                axisVel.plot(t, vel[0], color = 'orchid', linewidth = 1.4, label = 'J0')
                axisVel.plot(t, vel[1], color = 'orange', linewidth = 1.4, label = 'J1')
                axisVel.plot(t, vel[2], color = 'cornflowerblue', linewidth = 1.25, label = 'J2')
                axisVel.plot(t, vel[3], color = 'olivedrab', linewidth = 1.4, label = 'J3')
                axisVel.plot(t, vel[4], color = 'peru', linewidth = 1.4, label = 'J4')
                axisVel.plot(t, vel[5], color = 'crimson', linewidth = 1.4, label = 'J5')
                axisVel.plot(t, vel[6], color = 'royalblue', linewidth = 1.4, label = 'J6')
                axisVel.legend(('Joint 0', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'),
                            loc='lower right', ncol = 2, shadow = True, fancybox = True)
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
