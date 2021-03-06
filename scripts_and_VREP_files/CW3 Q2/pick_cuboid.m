%CW3Setup2;
original = [1.1503,0.5409,1.2167,-0.206,0];
%clear all;
%CW3Setup
base = GetObjectPosAndOrientation(clientID, vrep, 'youBotArmJoint0');
Rectangle14 = GetObjectPosAndOrientation(clientID, vrep, 'Cuboid2');
%%-----------------------Object Rect 14------------------------------------
%*******************Calculate the desired thetas first*********************
thetas = theta_finder(clientID,vrep,Rectangle14.position,'C2');
target_theta = thetas(37,1:5);
%%-----------------------Move to theta = 0---------------------------------
%Input to create trajectory function to get trajectory.
current = original;
data = zeros(2,5);
data(1,:) = current;
data(2,:) = [0,0,0,0,0];
% num of trajectory points = 21
sampling_rate = 200;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
GetJointPosition(getArmPosePub);
pause(0.5);
current = data(2,:);
%%-----------------------Move to desired theta1----------------------------
data = zeros(2,5);
data(1,:) = current;
data(2,:) = [target_theta(1),0,0,0,0];
sampling_rate = 200;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
pause(0.5);
current = data(2,:);
%%-----------------------Move to target thetas-----------------------------
%Input to create trajectory function to get trajectory.
data = zeros(2,5);
data(1,:) = current;
data(2,:) = target_theta;
% t = 1/20 = 0.05
sampling_rate = 200;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
grip;
current = data(2,:);
%%----------------------Move to thetas = 0 --------------------------------
data = zeros(2,5);
data(1,:) = current;
data(2,:) = zeros(1,5);
% t = 1/20 = 0.05
sampling_rate = 100;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
pause(10);