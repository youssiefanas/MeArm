#this is the same as maltab code but in python
# generate a gui that take the x,y,z 
# and return the angles of the servos
# and the angles of the joints
# and the position of the end effector
# and the position of the base
# and the position of the shoulder
# and the position of the elbow
# and the position of the gripper
#use tkinter for the gui
#use matplotlib for the plot
# use spatialmath for the transformation
# use roboticstoolbox for the robot
# use numpy for the math
# use math for the math
# use matplotlib for the plot
# use tkinter for the gui


from roboticstoolbox import * 
import roboticstoolbox as rtb
from spatialmath.base import *
from spatialmath import *
import numpy as np
import math
import matplotlib.pyplot as plt
import tkinter

j1_max = np.pi/2
j2_max = np.pi/8
j3_max = -(np.pi/8)-(np.pi/3)

j1_min = -np.pi/2
j2_min = np.pi*(11/16)
j3_min = -np.pi*(11/16)

# mapping
min_base=120
min_shoulder=70
min_elbow=140
min_gripper=50
max_base=10
max_shoulder=135
max_elbow=100
max_gripper=90
home_base=60
home_shoulder=90
home_elbow=140


def map_range(x, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    return (((x - old_min) * new_range) / old_range) + new_min

class MeArm (DHRobot):
    def __init__(self):
        deg= np.pi/180
        L0 = RevoluteDH(d = 7.5, a=10, alpha=np.pi/2, qlim=[-np.pi/2, np.pi/2])
        L1 = RevoluteDH(d = 0, a=9, alpha=0, qlim=[np.pi*(11/16), np.pi/8])
        L2 = RevoluteDH(d = 0, a=9, alpha=0, qlim=[-np.pi*(11/16), -(np.pi/8)-(np.pi/3)])
        super().__init__([L0, L1, L2], name='MeArm' , manufacturer = "E-JUST")
        self.home = np.array([0, np.pi/2, -np.pi/2])
        self.qz = np.zeros(3)
        self.min = np.array([-np.pi/2, np.pi*(11/16), -np.pi*(11/16)])
        self.max = np.array([np.pi/2 , np.pi/8, -(np.pi/8)-(np.pi/3)])
        self.addconfiguration("home", self.home)
        self.addconfiguration("qz", self.qz)



if __name__=="__main__":
    meArm= MeArm()
    print(meArm)

t_home=meArm.fkine(meArm.home)    
print(t_home)

tz=meArm.fkine(meArm.qz)
print(tz.t)


qt = rtb.jtraj(meArm.qz, meArm.home, 50)

# meArm.plot(qt.q, backend='pyplot', movie='mearm.gif')
# meArm.plot( qt.q)

# meArm.plot(meArm.max, dt=1)
# x=meArm.fkine(meArm.home)
# print(x)
# meArm.plot(meArm.home, dt=40 )    
# meArm.q=meArm.home
# meArm.teach()

# my_ws = np.array([])

# for i in range (1,50,1):
#     theta_1 = j1_min + (j1_max - j1_min) * np.random.rand()
#     theta_2 = j2_min + (j2_max - j2_min) * np.random.rand()
#     theta_3 = j3_min + (j3_max - j3_min) * np.random.rand()
#     q=[theta_1, theta_2 , theta_3]
#     print(q)
#     meArm.plot(q)
#     fk = meArm.fkine(q)

# g = transl(0,22.8,3.15)


print("Enter X, Y, Z Positons: ")

x= float(input("X from 0 to 24: "))
print(type(x))
while x not in range(0,24):
    print("X is out of Workspace")
    x= float(input("X: "))

y=  float(input("Y from -24 to 24: "))
while y not in range(-24,24):
    print("Z is out of Workspace")
    y= float(input("Y: "))

z= int(input("Z from 2, 21: "))
while z not in range(2  ,21):
    print("Z is out of Workspace")
    z= float(input("Z: "))


g=SE3(x,y,z)
print(g)
s = meArm.ikine_LM(g)

global servo_base, servo_elbow, servo_shoulder
print("s.q",s.q)
if s.q[0]  > -np.pi/2 and s.q[0] < np.pi/2 :
    print("1")
    if s.q[1] > np.pi/8  and s.q[1] <  np.pi*(11/16):
        print('2')
        if s.q[2] > -np.pi*(11/16) and  s.q[2] < (-(np.pi/8)-(np.pi/3)):
            print('3')
            print(s.q)
            meArm.plot(s.q,dt=10)
            gk = meArm.fkine(s.q)
            print("gk",gk)

            servo_base = map_range(s.q[0], j1_min,j1_max,min_base, max_base)
            servo_shoulder = map_range(s.q[1], j2_min, j2_max, min_shoulder, max_shoulder)
            servo_elbow = map_range(s.q[2], j3_min, j3_max, min_elbow, max_elbow)

            print("Servo Base: ",servo_base,"Servo shoulder: ", servo_shoulder,"Servo Elbow: ", servo_elbow)



