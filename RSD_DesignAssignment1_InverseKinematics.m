%Set the cartesian coordinate over here
x=550
y=150
z=20
roll=0.4
%D-H Parameter was set here
d1=427.7
d4=72.5
l1=400
l2=250
%This is the inverse kinematics calculation part
theta2=acos((x*x+y*y-l1*l1-l2*l2)/(2*l1*l2))
theta1=atan((y*(l1+l2*cos(theta2))-x*l2*sin(theta2))/(x*(l1+l2*cos(theta2))+y*l2*sin(theta2)))
d3=d1-d4-z
theta4=theta1+theta2-roll
% The links were define in this part
L1=Link([0 427.7 400 0])
L2=Link([0 0 250 pi])
L3=Link([0 0 0 0 1])
L3.qlim=[0 100]
L4=Link([0 72.5 0 0])
%Robot with 4 links were defined
robot=SerialLink([L1 L2 L3 L4])
%Plot the robot with specified joint angle and displacement
robot.plot([theta1 theta2 d3 theta4])
%Get the forward kinematics result from the 
%specified link joint angle and displacement
robot.fkine([theta1 theta2 d3 theta4])