#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
import matplotlib.cm as cm
import time
import math

#tf = 20  # final time
tau = np.arange(0., 1., 0.01)  # normalized time

'''

Ndof = int(input('Number of DOFs:'))
N = int(input('Number of waypoints:'))


#get all the waypoints
x_wp_str=[]
x0_dof_str=[]
xf_dof_str=[]
x_wp_dof_str=[]

tau_wp = [None]*N  # array with the time of each waypoint
x_wp=[] # waypoints of a single joint
x0_dof=[] # initial position of each joint
xf_dof=[] # final position of each joint
x_wp_dof=[] #  waypoints of all joints


## get final time
for d in range(Ndof):
    print('Joint',d+1)
    x0 = float(input('Inital position:'))
    x0_dof.append(x0)
    xf = float (input('Final Position:'))
    xf_dof.append(xf)
    x_wp.clear()
    for i in range(N):
        wp = input('Position of the waypoint: ')
        x_wp.append(float(wp))
    x_wp_dof.append(x_wp.copy())
# final time
tf = float(input('Final time:'))
print('Inital position:',x0_dof)
print('Final Position:',xf_dof)
print('Waypoints',x_wp_dof)


'''
#get all the waypoints
x_wp_str=[]
x0_dof_str=[]
xf_dof_str=[]
x_wp_dof_str=[]

x_wp=[]
tau_DOF = []
x0_dof=[]
xf_dof=[]
x_wp_dof=[]



count = 0
with open("/home/joao/ros_ws/devel/lib/motion_manager/Models/waypoints.txt") as fp:
    line = fp.readline()
    for line in fp:
        li=line.strip()
        if not li.startswith("#"):
            #print(li)
            if(count == 0):
                tf = float(line.rstrip().replace(',','.'))
            elif(count == 1):
                Ndof = int(line.rstrip())
            elif (count == 2):
                N = int(line.rstrip())

            elif(count == 3):
                x0_dof_str = line.rstrip().split(',')
                for i in range(len(x0_dof_str)):
                    x0_dof.append(float(x0_dof_str[i]))
            elif(count==4):
                xf_dof_str = line.rstrip().split(',')
                for i in range(len(xf_dof_str)):
                    xf_dof.append(float(xf_dof_str[i]))
            else:
                x_wp_str = line.rstrip().split(',')
                for i in range(len(x_wp_str)):
                    x_wp.append(float(x_wp_str[i]))
                x_wp_dof.append(x_wp.copy())
                x_wp.clear()
            count += 1

tau_wp = [None]*N

def write_sol_wp(time):
    # Writing to file
    with open("/home/joao/ros_ws/devel/lib/motion_manager/Models/waypoints_sol.txt", "w") as fp:
        for i in range(len(time)):
            fp.writelines(str(time[i])+'\n')


# get the equation (1-tau1)tau2 used to determine the Lagrange multiplier
def get_eq(tau_wp1,tau_wp2):
    eq2=0
    eq1 = (1 - tau_wp1) ** 5 * (-10 * tau_wp2 ** 3 + 15 * tau_wp2 ** 4 - 6 * tau_wp2** 5) + \
          (1 - tau_wp1) ** 4 * (20 * tau_wp2 ** 3 - 35 * tau_wp2 ** 4 + 15 * tau_wp2** 5) + \
          (1 - tau_wp1) ** 3 * (-10 * tau_wp2** 3 + 20 * tau_wp2 ** 4 - 10 * tau_wp2** 5)
    if tau_wp1 < tau_wp2:
        eq2 = (tau_wp2 - tau_wp1)**5

    return eq1+eq2

# lagrange multiplier is divided into 2 main parts: denominator and numertator --  pi = -120/tf^5 * num /den
# the denominator and numerator are also divided in two parts: first and second.
# the first part of booth algoritms is identical - function first_part
# the second has few changes, however, also quite identical  - function get_eq_den (denominator) and get_eq_num(numerator)


# input
#   tau_wp - array with the times of waypoints (where the changes are made)
#   tau_wp_or - array with the times of waypoints (with no changes) - tau_wp_or = [tau1,tau2,tau3.....]
# output:
#   array the first part of the algorithms - dimension of number of waypoints
def first_part(tau_wp,tau_wp_or):
    p1=[]
    i = len(tau_wp)
    n= len(tau_wp)
    while (i>0):
        if (i) % 2 == 0:
            res = -get_eq(tau_wp[i-1], tau_wp_or[n-1])
        else:
            res = get_eq(tau_wp[i-1], tau_wp_or[n-1])
        p1.append(res)
        i = i - 1
    return p1

# input
#   tau_wp - array with the times of waypoints (where the changes are made)
#   tau_wp_or - array with the times of waypoints (with no changes) - tau_wp_or = [tau1,tau2,tau3.....]
# output:
#  den - final denominator equation of the lagrange multiplier
def get_eq_den(tau_wp,tau_wp_or,den=0):
    n = len(tau_wp)
    if n == 0:
        return 1
    part1 = first_part(tau_wp,tau_wp_or)
    for i in range(len(tau_wp)):
        if (n > 1 and i>0): # change the order of the array tau_wp to get the entire expression of the denominator
            aux = tau_wp[n - 1 - i]
            tau_wp[n - 1 - i] = tau_wp[n - 1]
            tau_wp[n - 1] = aux
        # recursive function dictated by the size of array tau_wp
        den += part1[i]*get_eq_den(tau_wp[0:n-1].copy(),tau_wp_or[0:n-1].copy())
    return den


# input
#   tau_wp - array with the times of waypoints (where the changes are made)
#   tau_wp_or - array with the times of waypoints (with no changes) - tau_wp_or = [tau1,tau2,tau3.....]
#   x_wp - array with waypoints of joint i
#   xf - final position of joint i
#   x0 - initial position of joint i
# output:
#  num - final numerator equation of the lagrange multiplier
def get_eq_num(tau_wp_or,tau_wp,x_wp,xf,x0,num=0):
    n = len(tau_wp)
    if n == 1:
        return (xf-x0)*(10*tau_wp[0]**3-15*tau_wp[0]**4+6*tau_wp[0]**5)-(x_wp[0]-x0)
    part1 = first_part(tau_wp_or,tau_wp)
    for i in range(len(tau_wp)): ## cicle for the number of waypoints
        # change the order of the array tau_wp to get the entire expression of the numerator
        if (n > 1 and i>0):
            aux = tau_wp[n - 1 - i]
            tau_wp[n - 1 - i] = tau_wp[n - 1]
            tau_wp[n - 1] = aux
            aux = x_wp[n - 1 - i]
            x_wp[n - 1 - i] = x_wp[n-1]
            x_wp[n-1] = aux
        # recursive function dictated by the size of array tau_wp
        num += part1[i]*get_eq_num(tau_wp_or[0:n-1].copy(),tau_wp[0:n-1].copy(),x_wp[0:n-1].copy(),xf,x0)
    return num


# input
#   tau_wp - array with the times of waypoints (where the changes are made)
#   tau_wp_or - array with the times of waypoints (with no changes) - tau_wp_or = [tau1,tau2,tau3.....]
#   x_wp - array with waypoints of joint i
#   xf - final position of joint i
#   x0 - initial position of joint i
# output:
#  pi_num - array with the numerator equations of all lagrange multipliers
def get_pi_num(tau_wp,x_wp,xf,x0):
    n=len(tau_wp)
    pi_num=[]
    for k in range(len(tau_wp)):
        # Lagrange multiplier correspondent of the first waypoint- p1
        # p1 -> tau_wp = [tau1,tau2.---, tauN]
        #       x_wp = [wp1,wp2,....,wpN]
        # p2 -> tau_wp = [tau2,tau1.---, tauN]
        #       x_wp = [wp2,wp1,....,wpN]
        # p3 -> tau_wp = [tau3,tau1,tau2,..., tauN]
        #       x_wp = [wp3,wp1,wp2, ... ,wpN]
        # pN-> tau_wp = [tauN,tau1,tau2,..., tauN-1]
        #       x_wp = [wp3,wp1,wp2, ... ,wpN]
        if (n > 1 and k > 0):  # changes for p1-p2-p3-...-pn
            aux = tau_wp[0]
            tau_wp[0] = tau_wp[k]
            tau_wp[k] = aux
            aux = x_wp[0]
            x_wp[0] = x_wp[k]
            x_wp[k] = aux
        res = get_eq_num(tau_wp.copy(),tau_wp.copy(),x_wp.copy(),xf,x0)
        pi_num.append(res)
        #print(res)
        #print()
    return pi_num

# input
#   tau_wp - array with the times of waypoints
#   x_wp - array with waypoints of joint i
#   xf - final position of joint i
#   x0 - initial position of joint i
# output:
#  pi - array with all lagrange multiplier of all waypoints for 1 joint
def get_pi_eq(tau_wp,x_wp,xf,x0):
    pi=[]
    tau_wp_or=tau_wp.copy()
    #denominator
    pi_den=get_eq_den(tau_wp.copy(), tau_wp.copy())
    #numerator
    pi_num=get_pi_num(tau_wp.copy(), x_wp.copy(),xf,x0)
    for i in range(len(tau_wp)):
        ##get the final pi
        res=-120*pi_num[i]/(tf**5*pi_den)
        pi.append(res)
    return pi

# input
#   tau_wp - array with the times of waypoints (where the changes are made)
#   x_wp_dof - array with the waypoints of all joints
#   xf_dof - array with final positions of all joint
#   x0_dof - array with initial positions of all joint
# output:
#  eq - system of equations to determine the time of each waypoint
def get_equations(tau_wp,x_wp_dof,xf_dof,x0_dof):

   eq=[] # system of equations to determine the time of each waypoint
   energy_eq=[]
   part1 = 0
   part2 = 0
   part3 = 0

   pi_dof=[] # array with lagrange multipliers for each joint

   part_dof=[] # auxiliar array to determine the velocity of each joint

   # get pi's for every waypoint of each joint
   for k in range(Ndof):
       pi_dof.append(get_pi_eq(tau_wp.copy(),x_wp_dof[k].copy(),xf_dof[k],x0_dof[k]))
       for i in range(len(tau_wp)):
           part1 = pi_dof[k][i] * (1 - tau_wp[i]) ** 5 + part1
           part2 = pi_dof[k][i] * (1 - tau_wp[i]) ** 4 + part2
           part3 = pi_dof[k][i] * (1 - tau_wp[i]) ** 3 + part3
       part_dof.append([part1,part2,part3])


   for i in range(len(tau_wp)):
        v2=0
        eq_time=0
        # cicle to go through all joints
        for d in range(Ndof):
            for k in range(i):
                if(i==0):
                    v2 = 0 ## only v(tau1) does not have this part
                else:
                    v2 =  v2 + pi_dof[d][k]*(5*(tau_wp[i]-tau_wp[k])**4)

            # joint velocity at the waypoint
            v_wp =(xf_dof[d]-x0_dof[d])*(30*tau_wp[i]**2-60*tau_wp[i]**3 + 30*tau_wp[i]**4)/tf + (tf**5/120)/tf*(\
                            (part_dof[d][0])*(-30*tau_wp[i]**2 + 60*tau_wp[i]**3-30*tau_wp[i]**4) +\
                            (part_dof[d][1])*(60*tau_wp[i]**2 -140*tau_wp[i]**3 +75*tau_wp[i]**4) +\
                            (part_dof[d][2])*(-30*tau_wp[i]**2 + 80*tau_wp[i]**3-50*tau_wp[i]**4) +\
                            + v2)
            # equation of system of equations
            #  waypoint1 : p1*v_wp1 + p2*v_wp2 + .... + pn*v_wpn
            eq_time= eq_time + pi_dof[d][i]*v_wp
        #system of equations
        eq.append(eq_time)

   return eq


#input
    #tau_wp - time of each waypoint
    #pi  - lagrange multiplier of each waypoint for all joints
    #xf_dof - final position of all joints
    #x0_dof - initial position of all joints
#output
    # x_minus,v_minus,acc_minus - joints position, velocity, acceleration of interval t0<t<t1 (initial to first waypoint)
def get_eq_minus(tau_wp,pi,xf_dof,x0_dof):

    part1 = 0
    part2 = 0
    part3 = 0

    for i in range(len(tau_wp)):
       part1 = pi[i] * (1 - tau_wp[i]) ** 5 + part1
       part2 = pi[i] * (1 - tau_wp[i]) ** 4 + part2
       part3 = pi[i] * (1 - tau_wp[i]) ** 3 + part3

    # position before the way point (0<=t<=t1
    x_minus = x0_dof + (xf_dof-x0_dof)*(10*tau**3-15*tau**4 + 6*tau**5) + (tf**5/120)* (\
                    (part1)*(-10*tau**3 + 15*tau**4-6*tau**5) +\
                    (part2)*(20*tau**3 -35*tau**4 +15*tau**5) +\
                    (part3)*(-10*tau**3 + 20*tau**4-10*tau**5))

    # velocity before the way point (0<=t<=t1)
    v_minus =(xf_dof-x0_dof)*(30*tau**2-60*tau**3 + 30*tau**4)/tf + (tf**5/120)/tf*(\
                    (part1)*(-30*tau**2 + 60*tau**3-30*tau**4) +\
                    (part2)*(60*tau**2 -140*tau**3 +75*tau**4) +\
                    (part3)*(-30*tau**2 + 80*tau**3-50*tau**4) )

    # acceleration before the way point (0<=t<=t1)
    acc_minus =(xf_dof-x0_dof)*(60*tau-180*tau**2 + 120*tau**3)/tf**2 + (tf**5/120)/tf**2*(\
                    (part1)*(-60*tau + 180*tau**2-120*tau**3) +\
                    (part2)*(120*tau -420*tau**2 +300*tau**3) +\
                    (part3)*(-60*tau + 240*tau**2-200*tau**3))

    return x_minus,v_minus,acc_minus

# return an array with every trajectory
# ex: x_plus[0] = x_minus
#     x_plus[1] = t1 < t < t2
#     x_plus[2] = t2 < t < t3
#input
    #tau_wp - time of each waypoint
    #pi  - lagrange multiplier of each waypoint for all joints
    #xf_dof - final position of all joints
    #x0_dof - initial position of all joints
#output
    # x_plus, v_plus, acc_plus - joints position, velocity, acceleration of all trajectory segmentations after first waypoint (t1 < t < t2;  t2 < t < t3 ; ... ; t[N-1] < t < tN)
def get_eq_plus(tau_wp,pi,xf_dof,x0_dof):
    x_plus = [] # joints position
    v_plus = [] # joints velocity
    acc_plus = [] # joints acceleration

    # position, velocity and acceleration of interval t0 < t < t1
    x_minus,v_minus,acc_minus = get_eq_minus(tau_wp,pi,xf_dof,x0_dof)

    for i in range(len(tau_wp)):
        if(len(tau_wp)==1 or i==0):
            x_plus.append(x_minus + pi[0] * tf** 5 * (tau - tau_wp[i]) ** 5 / 120)
            v_plus.append(v_minus + 5*pi[0] *tf** 4 * (tau - tau_wp[i]) ** 4 / 120)
            acc_plus.append(acc_minus + 20*pi[0]*tf ** 3 * (tau - tau_wp[i]) ** 3 / 120)
        else:
            x_plus.append(x_plus[i-1] + pi[i] * tf**5*(tau-tau_wp[i])**5/120)
            v_plus.append(v_plus[i-1] + 5*pi[i] * tf**4*(tau-tau_wp[i])**4/120)
            acc_plus.append(acc_plus[i-1] + 20*pi[i]* tf**3*(tau-tau_wp[i])**3/120)

    return x_plus, v_plus, acc_plus

#input
    #tau_wp - time of each waypoint
    #pi  - lagrange multiplier of each waypoint for all joints
    #xf_dof - final position of all joints
    #x0_dof - initial position of all joints
#output
    # pos, vel, acc - joints position, velocity, acceleration of all trajectory segmentations
def compose(tau_wp,pi,xf_dof,x0_dof):
    # joints trajectory, velocity and acceleration  of interval t0 < t < t1
    x_minus,v_minus,acc_minus = get_eq_minus(tau_wp,pi,xf_dof,x0_dof)
    # joints trajectory, velocity and acceleration  of interval t1 < t < t2;  t2 < t < t3 ; ... ; t[N-1] < t < tN
    x_plus,v_plus,acc_plus = get_eq_plus(tau_wp,pi,xf_dof,x0_dof)
    pos = []
    vel = []
    acc = []
    time=[]
    k=0
    wp_pos_res=[]
    for i in range(len(tau)):
        if tau[i] <= tau_wp[0]:
            pos.append(x_minus[i])
            vel.append(v_minus[i])
            acc.append(acc_minus[i])
        else:
            pos.append(x_plus[k-1][i])
            vel.append(v_plus[k-1][i])
            acc.append(acc_plus[k-1][i])

        if (k<len(tau_wp)):
            if ((tau[i] - tau_wp[k]) ** 2) <= 0.0001:
                time.append(i)  ## save the composed time of every waypoint
                k=k+1

    for i in range(len(tau_wp)):
        print('Position at the',i+1,' waypoint ( tau',i+1,'= ',tau_wp[i],') = ',pos[time[i]])
        wp_pos_res.append(pos[time[i]])
    print()
    for i in range(len(tau_wp)):
        print('Velocity at the',i+1,'waypoint ( tau',i+1,'= ',tau_wp[i],') = ',vel[time[i]])
    print()
    for i in range(len(tau_wp)):
        print('Acceleration at the',i+1,' waypoint ( tau',i+1,'= ',tau_wp[i],') = ',acc[time[i]])

    return pos,vel,acc,wp_pos_res


#input
    #root - time of each waypoint
    #x_wp_dof  - waypoints of all joints
    #xf_dof - final position of all joints
    #x0_dof - initial position of all joints
#output
    # norm - energy of the system of equations
def root_energy(root,x_wp_dof,xf_dof,x0_dof):
    norm=0
    res=get_equations(root,x_wp_dof,xf_dof,x0_dof)
    for i in range(len(res)):
        norm= norm + res[i]**2
    return np.sqrt(norm)

# function to select only acceptable roots
# conditions:
# t1 < t2 < ... < tN
# t1>0
#tN < 1
def check_root(root, roots ):
    for j in range(len(root)):
        #check if the root lies between the range [0,1]
        if (root[j]<0.98 and root[j]>0.1):
            # check if the root is repetitive
            for k in range(len(root)):
                if k!= j: #make sure that dont compare the same value
                    if round(root[j],2)==round(root[k],2):
                        return False
        else:
            return False
    # check if the tauN < tauN+1
    for k in range(len(root) - 1):
        if root[k] > root[k + 1]:
            return False
    # I also need to verify if this acceptable root is not repetitive in the acceptables roots:
    aux=True
    for i in range(len(roots)):
        for k in range(len(root)):
            if round(roots[i][k],2) != round(root[k],2):
                aux=False
        if aux==True:return False

    return True

#choose the root that minimises the energy of the system
# in principle the system returns only one acceptable root
def choose_root(energy,roots,init_guess):
    ## get the minimum energy value
    min_energy=min(energy)
    min_index = energy.index(min_energy)
    return roots[min_index],init_guess[min_index]


# function to get all possible initial guesses between 0 and 1 in range of steps "step"
# never used - just to do some tests
def get_all_init_guess(n,init_guess,all_init_guess,time,step):
    if(n<=1):
        while (time+step) < 0.99:
            time = time + step
            init_guess.append(time)
            all_init_guess.append(init_guess.copy())
            del init_guess[-1]
        return init_guess, all_init_guess
    while (time+step)<0.99:
        time = time + step
        init_guess.append(time)
        init_guess, all_init_guess = get_all_init_guess(n - 1, init_guess,all_init_guess, time,step)
        del init_guess[-1]

    return init_guess,all_init_guess

#input
    #x_wp - waypoints
    #xf - final position
    #x0 - initial position
#output
    # init_guess - array of initial guesses for each waypoint time
def get_init_guess(x_wp,xf,x0):

    sum_traj = 0
    traj_wp=[] # path travelled by the robot for each wp
    init_guess=[]

    # calculate the travelled cost
    for i in range(len(x_wp)+1):
        if i==0:
            sum_traj = sum_traj + abs(x0-x_wp[0])
            traj_wp.append(sum_traj)
        elif i==len(x_wp):
            sum_traj = sum_traj + abs(xf-x_wp[-1])
        else:
            sum_traj = sum_traj + abs(x_wp[i]-x_wp[i-1])
            traj_wp.append(sum_traj)

    ## calculate an aproximate initial guess
    for i in range(len(x_wp)):
        if(sum_traj>0):
            guess=traj_wp[i]/sum_traj
        else:
            guess=0
        init_guess.append(guess)


    return init_guess

#input
    #x_wp_dof - waypoints of all joints
    #xf_dof - final position of all joints
    #x0_dof - initial position of all joints
#output
    # init_guess - array of initial guesses for each waypoint considering all joints
def get_init_guess_dof(x_wp_dof, xf_dof, x0_dof):
    init_guess_dof = []
    init_guess = []
    sum = 0
    # get a init guess for each DOF
    for i in range(Ndof):
        init_guess_dof.append(get_init_guess(x_wp_dof[i], xf_dof[i], x0_dof[i]))
    # sum the init guess of all joints and do the average to get the final init guess
    for k in range(N):
        for i in range(Ndof):
            sum = sum + init_guess_dof[i][k]
        init_guess.append(sum / Ndof)
        sum = 0
    return init_guess


# if the initial guess is not sufficient for the function fsolve to find an acceptable root,
# this function returns more initial guesses with range of +/- 5%
def all_init_guess(init_guess):
    all_init_guess = []
    init_guess_first=[]
    init_guess_low=[]
    init_guess_high=[]
    init=[]
    ## set more init guess in a range of +/- 0.1 for every waypoint
    all_init_guess.append(init_guess)
    for i in range(len(init_guess)):
        init.append(round(init_guess[i]-0.05,1))
        ## 2 more initial guesses in a range +0.05 and -0.05 for every waypoint
        if init_guess[i]>0.1:
            init_guess_low.append(init_guess[i]-0.05)
        else:init_guess_low.append(init_guess[i])
        if init_guess[i]<0.9:
            init_guess_high.append(init_guess[i]+0.05)
        else:init_guess_high.append(init_guess[i])

    all_init_guess.append(init_guess_low)
    all_init_guess.append(init_guess_high)
    all_init_guess.append(init)
    return all_init_guess



########################################################################################################################
########              MAIN
########################################################################################################################
start_time = time.time()

init_guess = [] # array of initial guesses for the function fsolve
roots=[] # roots of the system of equations
energy=[]
accept_guess=[]
solution=False
# get initial guess by doing the average of all joints
init_guess_dof = get_init_guess_dof(x_wp_dof, xf_dof, x0_dof)
init_guess.append(init_guess_dof)

while solution==False:
    for j in range(len(init_guess)):
        ## get the solutions of the system with fsolve
        root = fsolve(get_equations, init_guess[j],args=(x_wp_dof,xf_dof,x0_dof))
        if(check_root(root,roots)==True):
            energy.append(root_energy(root,x_wp_dof,xf_dof,x0_dof))
            roots.append(root)
            accept_guess.append(init_guess[j])
            solution=True
        else:
            ## get more init guesses
            init_guess.clear()
            init_guess = all_init_guess(init_guess_dof)

# if fsolve returns more than one acceptable solution
# pick the one that minimises the energy of the system
root=0
init_guess=0
root,init_guess=choose_root(energy,roots,accept_guess)

print()
print('selected root :')
print('guess',init_guess)
print('root',root)
print()

write_sol_wp(root)

'''
pos_dof=[] ## array of joints position trajectory for all joints
vel_dof=[] ##  array of joints velocity for all joints
acc_dof=[] ##  array of joints acceleration for all joints
wp_pos_res=[] ## array of joints position at waypoints time

for i in range(Ndof):
    # get all lagrange multipliers of joint i
    pi_value = get_pi_eq(root,x_wp_dof[i],xf_dof[i],x0_dof[i])
    print()
    print('###################')
    print('Joint',i+1)
    ## get trajectory,velocity and acceleration of every joint
    pos,vel,acc,wp_pos = compose(root,pi_value,xf_dof[i],x0_dof[i])
    pos_dof.append(pos)
    vel_dof.append(vel)
    acc_dof.append(acc)
    wp_pos_res.append(wp_pos)

# execution time
print("--- %s seconds ---" % (time.time() - start_time))

###############
### plots #####
###############

##define the colors for the plots
x = np.arange(10)
ys = [i+x+(i*x)**2 for i in range(10)]
colors = cm.rainbow(np.linspace(0, 1, len(ys)))
# plots
## plot each DOF in differents plots
for i in range(Ndof):
    #max and min values for the plot scale
    max1=max(pos_dof[i])+max(pos_dof[i])/4
    min1=min(pos_dof[i])+min(pos_dof[i])/4

    if(max(vel_dof[i])>max(acc_dof[i])):max2=max(vel_dof[i])+max(vel_dof[i])/4
    else:max2=max(acc_dof[i])+max(acc_dof[i])/4

    if(min(vel_dof[i])<min(acc_dof[i])):min2=min(vel_dof[i])+min(vel_dof[i])/4
    else:min2=min(acc_dof[i])+min(acc_dof[i])/4

    fig=plt.figure(i)
    plt.suptitle('Joint{}'.format(i+1), fontsize=16)
    #position
    ax1 = plt.subplot(211)
    line1=ax1.plot(tau,pos_dof[i],linestyle='solid',color='black', label='position')
    ax1.legend(loc='best')
    #velocity and acceleration
    ax2 = plt.subplot(212)
    ax2.plot(tau,vel_dof[i],linestyle='solid',color='red', label='velocity')
    ax2.plot(tau,acc_dof[i],linestyle='solid',color='blue', label='acceleration')
    ax2.set_xlabel('tau')
    ax2.legend(loc='best')
    #ax1.set_ylim([min1,max1])
    #ax2.set_ylim([min2,max2])
    ax1.set_xlim([0,1])
    ax2.set_xlim([0,1])
    count=0
    for tau_i,xwp_i,c in zip(root,wp_pos_res[i], colors):
        count+=1
        ax1.axvline(x=tau_i, color=c,label='x{} = {}'.format(count,round(xwp_i,2)), linestyle='--')
        ax2.axvline(x=tau_i, color=c,label='x{} = {}'.format(count,round(xwp_i,2)), linestyle='--')

    ax1.legend(loc='best')
    ax1.axhline(linewidth=0.5, color='black')
    ax2.axhline(linewidth=0.5, color='black')
    fig.show()

input()
'''
