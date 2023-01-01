close all
clear
clc

L(1) = Link([0 7.5 2+8 pi/2 0]);
L(2) = Link([0 0 9 0 ]);
L(3) = Link([0 0 9 0 ]);
% L(4) = Link([0 0 7.5 0]);
me_arm = SerialLink(L,'name', 'me_arm');
me_arm

%range of motion        corresponding servo readings
%j1= -pi/2:pi/2         S1= 
%j2=pi/2:pi             S2=
%j_3=0:-pi/4
%j3=-pi/2: 13*pi/18     S3=

j1_home=0;
j1_min=-pi/2;
j1_max=pi/2;

j2_home=pi/2;
j2_min=11*pi/16;
j2_max=pi/8;

j3_home=-j2_home;
j3_min=-j2_min;
j3_max=-j2_max-(pi/3);
%j3=-j2+(j_3);

j4_home=-j2_home-j3_home;
j4_min=-j2_min-j3_min;
j4_max=-j2_max-j3_max;

% j4=-j2-j3;


%home
j_home=[j1_home j2_home j3_home ];
me_arm.plot(j_home)
pause(3)
hold on
%max
j_max=[j1_max j2_max j3_max];
me_arm.plot(j_max)
pause(3)

j_min=[j1_min j2_min j3_min];
me_arm.plot(j_min)
pause(3)

%%
for i=1:1000000
%you can generate N random numbers in the interval (a,b) with the formula r = a + (b-a).*rand(N,1).
%next step is to: define a range for each joint to move with, as this will
%be it max and min range of motion

theta_1=j1_min+(j1_max-j1_min)*rand();
theta_2=j2_min+(j2_max-j2_min)*rand();
theta_3=j3_min+(j3_max-j3_min)*rand();


m=me_arm.fkine([theta_1 theta_2 theta_3 ]);
%get the forward kinematics for each joint to get transformation matrix ,
% as we call it matrix m

me_arm.plot([theta_1 theta_2 theta_3]) %you can uncomment this

%from the matrix m we will extract the translation values as in matrix x
%(robot positions)
x(i,:)=(m.t)';
%the x variable stores the values x,y,z positions for a certain transfromation

s(i,:)=me_arm.ikine(m, [0 0 0], 'mask' ,[1 1 1 0 0 0]);
%make inverse kinematix for the output transformation to get and your joints
%values in matrix s 
%the s variable stores the joints values to make a certain rotaion


plot3(x(i,1), x(i,2), x(i,3),'o'); %you can uncomment this line and comment
% the next line

end
%%
%enter the positions

x=  0;
y=  22.8;
z= 3.15;
g=[1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
me_i=me_arm.ikine(g, [0 0 0], 'mask' ,[1 1 1 0 0 0]);
me_arm.plot(me_i);
j1=me_i(1)
j2=me_i(2)
j3=me_i(3)

%%
%Servo map
min_base=120;
min_shoulder=70;
min_elbow=140;
min_gripper=50;
max_base=10;
max_shoulder=135;
max_elbow=100;
max_gripper=90;
home_base=60;
home_shoulder=90;
home_elbow=140;


servo_base=interp1([j1_min,j1_max],[min_base,max_base],me_i(1))
servo_elbow=interp1([j2_min,j2_max],[min_elbow,max_elbow],me_i(2))
servo_shoulder=interp1([j3_min,j3_max],[min_shoulder,max_shoulder],me_i(3))

%%
servo_publisher= rospublisher("/matlab_joints","geometry_msgs/Vector3")
joints_sent=rosmessage(servo_publisher);
joints_sent.X=servo_base;
joints_sent.Y=servo_shoulder
joints_sent.Z=servo_elbow
send(servo_publisher,joints_sent)