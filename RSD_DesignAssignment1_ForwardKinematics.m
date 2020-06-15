% Can set the joint angle over here
theta1=0.1
theta2=0.2
d3=30
theta4=0.7
%D-H Parameter was set here
d1=427.7
d4=72.5
l1=400
l2=250
%This is the forward kinematics calculation part
Xx=cos(theta4)*cos(theta1+theta2)+sin(theta4)*sin(theta1+theta2)
Yx=-sin(theta4)*cos(theta1+theta2)+cos(theta4)*sin(theta1+theta2)
Zx=0
Xy=cos(theta4)*sin(theta1+theta2)-sin(theta4)*cos(theta1+theta2)
Yy=-sin(theta4)*sin(theta1+theta2)-cos(theta4)*cos(theta1+theta2)
Zy=0
Xz=0
Yz=0
Zz=-1
Px=l2*cos(theta1+theta2)+l1*cos(theta1)
Py=l2*sin(theta1+theta2)+l1*sin(theta1)
Pz=d1-d3-d4
%Show the forward kinematics in matrix form
A=[Xx Yx Zx Px;Xy Yy Zy Py;Xz Yz Zz Pz;0 0 0 1]
roll=asin(cos(theta4)*sin(theta1+theta2)-sin(theta4)*cos(theta1+theta2))