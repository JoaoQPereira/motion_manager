#!/usr/bin/env python
import rospy
import os
import errno
import time
import Tkinter as tk
import tkFont
import tkMessageBox
from sensor_msgs.msg import JointState

# ----------------------------------------------------------------------------------------------------------- #
#                                              Class JointStates                                              #
# ----------------------------------------------------------------------------------------------------------- #
class JointStates:
    # ****************************** #
    #         FUNCTION: init         #
    # ****************************** #
    def __init__(self):
        print 'JointStates'
        self.joints_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6']
        self.joints_pos = [[] for i in range(0, 7)]
        self.joints_vel = [[] for i in range(0, 7)]

        self.configFiles()

    # *************************************** #
    #         FUNCTION: subscribeNode         #
    # *************************************** #
    def subscribeTopic(self):
        print 'subscribeNode'
        self.sub = rospy.Subscriber("/robot/joint_states", JointState, self.callbackJoints)

    # *************************************** #
    #         FUNCTION: subscribeNode         #
    # *************************************** #
    def unsubscribeTopic(self):
        print 'unsubscribeNode'
        if self.sub:
            self.sub.unregister()

    # **************************************** #
    #         FUNCTION: callbackJoints         #
    # **************************************** #
    def callbackJoints(self, data):
        #
        print 'callbackJoints'
        for allJoints_name in data.name:
            #
            if allJoints_name in self.joints_names:
                #
                i = data.name.index(allJoints_name) - 1
                #
                self.joints_pos[i].append(data.position[i + 1])
                #
                self.joints_vel[i].append(data.velocity[i + 1])

        #
        time.sleep(0.2)

    # ************************************* #
    #         FUNCTION: configFiles         #
    # ************************************* #
    def configFiles(self):
        print 'configFiles'
        #
        filePos = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/joints_pos.text'
        fileVel = '/home/sara/catkin_ws/devel/lib/motion_manager/results/robot/joints_vel.text'
        filesNames = [filePos, fileVel]

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
            with open(file, "w") as f:
                #
                if file == filePos:
                    f.write("# JOINTS POSITION \n")
                    f.write("# Time [s], T_0 [rad], T_1 [rad], T_2 [rad], T_3 [rad], T_4 [rad], T_5 [rad], T_6 [rad]\n")
                #
                else:
                    f.write("# JOINTS VELOCITY \n")
                    f.write("# Time [s], T_0 [rad/s], T_1 [rad/s], T_2 [rad/s], T_3 [rad/s], T_4 [rad/s], T_5 [rad/s], T_6 [rad/s]\n")
                #
                f.close()


# ----------------------------------------------------------------------------------------------------------- #
#                                              Class Application                                              #
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
        root = tk.Tk()
        root.title("[Sawyer Robot]: Monitoring of the Movements")
        root.geometry("550x375")
        root.configure(background = 'gainsboro')

        #
        fontTitle = tkFont.Font(family = 'Helvetica', size = 12, weight = 'bold')
        fontButton = tkFont.Font(family = 'system', size = 10)

        #***************************
        recordFrame = tk.Frame(root, width = 50, height = 50, background = 'white')
        recordLabel = tk.Label(recordFrame, text = "Record the movement to be executed", font = fontTitle, background = 'white')
        recordLabel.pack(pady = 10)
        #
        buttonStart = tk.Button(recordFrame, text = 'Start', command = self.bStart)
        buttonStart.config(height = 2, width = 12, bg = 'indianred', font = fontButton)
        buttonStart.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')
        #
        buttonStop = tk.Button(recordFrame, text = 'Stop', command = self.bStop)
        buttonStop.config(height = 2, width = 12, bg = 'lightslategray',  font = fontButton)
        buttonStop.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')
        #
        buttonClear = tk.Button(recordFrame, text = 'Clear', command = self.bClear)
        buttonClear.config(height = 2, width = 12, bg = 'peru', font = fontButton)
        buttonClear.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')

        #***************************
        resultsFrame = tk.Frame(root, width = 50, height = 50, background = 'white')
        resultsLabel = tk.Label(resultsFrame, text = "Show the results of the:", font = fontTitle, background = 'white')
        resultsLabel.pack(pady = 10)
        #
        buttonJoints = tk.Button(resultsFrame, text = 'Joints', command = self.bJoints)
        buttonJoints.config(height = 2, width = 12, bg = 'goldenrod', font = fontButton)
        buttonJoints.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')
        #
        buttonHand = tk.Button(resultsFrame, text = 'Hand', command = self.bHand)
        buttonHand.config(height = 2, width = 12, bg = 'darkkhaki',  font = fontButton)
        buttonHand.pack(side = 'left', padx = 12, pady = 10, expand = 'yes')

        #***************************
        recordFrame.pack(side = 'top', pady = 50, fill = 'none', expand = 'no')
        resultsFrame.pack(side = 'top', fill = 'none', expand = 'no')
        root.mainloop()

    # ******************************* #
    #         FUNCTION: bStop         #
    # ******************************* #
    def bStop(self):
        print 'Stop Button'
        if self.run:
            self.stop = 1
            self.jointStates.unsubscribeTopic()

    # ******************************** #
    #         FUNCTION: bClean         #
    # ******************************** #
    def bClear(self):
        print 'Clear Button'
        self.clear = 1

    # ******************************** #
    #         FUNCTION: bStart         #
    # ******************************** #
    def bStart(self):
        print 'Start Button'
        if not self.run:
            self.start = 1
            self.run = 1
            self.jointStates.subscribeTopic()

    # ********************************* #
    #         FUNCTION: bJoints         #
    # ********************************* #
    def bJoints(self):
        print 'Joints Button'

    # ********************************* #
    #         FUNCTION: bJoints         #
    # ********************************* #
    def bHand(self):
        print 'Hand Button'


# ----------------------------------------------------------------------------------------------------------- #
#                                                    Main                                                     #
# ----------------------------------------------------------------------------------------------------------- #
def main():
    rospy.init_node('monitoring_interface')
    app = Application()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
