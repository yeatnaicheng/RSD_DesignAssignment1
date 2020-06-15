%Set the cartesian coordinate over here
x=636.84
y=113.81
z=325.20
roll=-0.4
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
