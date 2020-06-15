% Set the hold function to be on so that we can trace the path
hold on
%Set the cartesian coordinate over here
x=550
y=150
z=20
Ax=250
Ay=150
Az=20
Bx=250
By=-150
Bz=20
Cx=550
Cy=-150
Cz=20
Dx=550
Dy=150
Dz=20
Ex=250
Ey=150
Ez=320
Fx=250
Fy=-150
Fz=320
Gx=550
Gy=-150
Gz=320
Hx=550
Hy=150
Hz=320
rollDegree=0

%Create Variable for keep track on the current joint variables
Currenttheta1=0
Currenttheta2=0
Currentd3=0
Currenttheta4=0
CurrentJoint=[Currenttheta1 Currenttheta2 Currentd3 Currenttheta4]

%Create Variable for keep track on the current coordinate variables
%The initialised value is when joint angles and distance are all 0
%Pre-calculated
CurrentX=650
CurrentY=0
CurrentZ=185.2
CurrentRoll=0
CurrentCoordinate=[CurrentX CurrentY CurrentZ CurrentRoll]

%Define Robot Links
L1=Link([0 427.7 400 0])
L2=Link([0 0 250 pi])
L3=Link([0 0 0 0 1])
L3.qlim=[0 180] % Set the limit of the robot to 180 movement in verticle axis
L4=Link([0 72.5 0 0])
robot=SerialLink([L1 L2 L3 L4])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Start Calling Functiono to Do the calculation and plotting
%Plot the initial robot position and get the coordinate
CurrentCoordinate=PlotInitial(robot)
CurrentCoordinate=PlotStart(robot,CurrentCoordinate,Ax,Ay,Az,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Bx,By,Bz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Cx,Cy,Cz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Dx,Dy,Dz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Ax,Ay,Az,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Ex,Ey,Ez,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Fx,Fy,Fz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Bx,By,Bz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Fx,Fy,Fz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Gx,Gy,Gz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Cx,Cy,Cz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Gx,Gy,Gz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Hx,Hy,Hz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Dx,Dy,Dz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Hx,Hy,Hz,rollDegree)
CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,Ex,Ey,Ez,rollDegree)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function CurrentCoordinate=PlotInitial(robot)
robot.plot([0,0,0,0])
CurrentCoordinate=ForwardKinematic(0,0,0,0)
end
function CurrentCoordinate=PlotStart(robot,CurrentCoordinate,x,y,z,roll)

%Do the inverse kinematics
IKSolution=calculateInverseKinematic(x,y,z,roll)
robot.plot([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)])
%Update the current coordinate
CurrentX=x
CurrentY=y
CurrentZ=z
CurrentRoll=roll
%Return the new current coordinate
CurrentCoordinate=ForwardKinematic(IKSolution(1),IKSolution(2),IKSolution(3),IKSolution(4))
end
function CoordinateResult=ForwardKinematic(theta1,theta2,d3,theta4)
d1=427.7
d4=72.5
l1=400
l2=250

Px=l2*cos(theta1+theta2)+l1*cos(theta1)
Py=l2*sin(theta1+theta2)+l1*sin(theta1)
Pz=d1-d3-d4
roll=rad2deg(asin(cos(theta4)*sin(theta1+theta2)-sin(theta4)*cos(theta1+theta2)))
CoordinateResult=[Px Py Pz roll]
end
function IKSolution=calculateInverseKinematic(x,y,z,roll)
%Constant D-H Parameter
d1=427.7
d4=72.5
l1=400
l2=250

%Convert the received roll angle from degree to radian
roll=degtorad(roll)

%Inverse Kinematic Calculation
theta2=acos((x*x+y*y-l1*l1-l2*l2)/(2*l1*l2))
theta1=atan((y*(l1+l2*cos(theta2))-x*l2*sin(theta2))/(x*(l1+l2*cos(theta2))+y*l2*sin(theta2)))
d3=d1-d4-z
theta4=theta1+theta2-roll
IKSolution=[theta1 theta2 d3 theta4]
end
function PlotGoToCoordinatea()
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
end
function CurrentCoordinate=PlotGoToCoordinate(CurrentJoint,robot,CurrentCoordinate,x,y,z,roll)

pausetime=0.1


%Save the current coordinate separately in each variable for visual purpose
CurrentX=CurrentCoordinate(1)
CurrentY=CurrentCoordinate(2)
CurrentZ=CurrentCoordinate(3)
CurrentRoll=CurrentCoordinate(4)

%Move in X Axis
if x>CurrentX
    Change=10
else
    Change=-10
end

for a=CurrentX:Change:x
    CurrentX=a;
    IKSolution=calculateInverseKinematic(CurrentX,CurrentY,CurrentZ,CurrentRoll)
    robot.plot([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)])
    %Trace the path
    FKMatrix=robot.fkine([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)]).T
    plot2([CurrentCoordinate(1) CurrentCoordinate(2) CurrentCoordinate(3);FKMatrix(1,4) FKMatrix(2,4) FKMatrix(3,4)],"Red")
    pause(pausetime)
end
%Move in Y Axis
if y>CurrentY
    Change=10
else
    Change=-10
end
for a=CurrentY:Change:y
    CurrentY=a;
    IKSolution=calculateInverseKinematic(CurrentX,CurrentY,CurrentZ,CurrentRoll)
    robot.plot([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)])
    %Trace the path
    FKMatrix=robot.fkine([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)]).T
    plot2([CurrentCoordinate(1) CurrentCoordinate(2) CurrentCoordinate(3);FKMatrix(1,4) FKMatrix(2,4) FKMatrix(3,4)],"Red")
    pause(pausetime)
end

%Move in Roll
if roll>CurrentRoll
    Change=1
else
    Change=-1
end
for a=CurrentRoll:Change:roll
    CurrentRoll=a;
    IKSolution=calculateInverseKinematic(CurrentX,CurrentY,CurrentZ,CurrentRoll)
    robot.plot([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)])
    pause(pausetime)
end

%Move in Z Axis
if z>CurrentZ
    Change=10
else
    Change=-10
end
for a=CurrentZ:Change:z
    CurrentZ=a;
    IKSolution=calculateInverseKinematic(CurrentX,CurrentY,CurrentZ,CurrentRoll)
    robot.plot([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)])
    %Trace the path
    FKMatrix=robot.fkine([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)]).T
    plot2([CurrentCoordinate(1) CurrentCoordinate(2) CurrentCoordinate(3);FKMatrix(1,4) FKMatrix(2,4) FKMatrix(3,4)],"Red")
    pause(pausetime)
end
%Return the new current coordinate
CurrentCoordinate=[CurrentX CurrentY CurrentZ CurrentRoll]
end