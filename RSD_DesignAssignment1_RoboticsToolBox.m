% Can set the joint angle over here
theta1=0.1
theta2=0.2
d3=30
theta4=0.7
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
