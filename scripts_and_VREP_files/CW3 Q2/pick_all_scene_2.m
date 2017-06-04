CW3Setup2;
original = [1.1503,0.5409,1.2167,-0.206,0];
base = GetObjectPosAndOrientation(clientID, vrep, 'youBotArmJoint0');
%%-------Calculation------------------
Rectangle15 = GetObjectPosAndOrientation(clientID, vrep, 'Rectangle15');
thetas = theta_finder(clientID,vrep,Rectangle15,'R15');
target_theta_15 = thetas(1,1:5);

Rectangle14 = GetObjectPosAndOrientation(clientID, vrep, 'Rectangle14');
Rectangle13 = GetObjectPosAndOrientation(clientID, vrep, 'Rectangle13');
destination = Rectangle13.position;
destination(3) = Rectangle13.position(3) + 2 * Rectangle14.bbox_max(3) + 0.01;
dest_theta_15 = destination_theta(clientID,vrep,destination,'R15');
dest_theta_15 = dest_theta_15(1,1:5);

destination = Rectangle13.position;
destination(3) = Rectangle13.position(3) + 4 * Rectangle14.bbox_max(3) + 0.02;
dest_theta_14 = destination_theta(clientID,vrep,destination,'R14');
dest_theta_14 = dest_theta_14(1,1:5);

SendJointTargetPosition(setPoseArmPub,original);

current = original;
%%--------------offset---------Move to desired theta1----------------------------
data = zeros(2,5);
data(1,:) = current;
data(2,:) = [target_theta_15(1),0,0,0,0];
sampling_rate = 200;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
current = data(2,:);
%%-----------------------Move to target thetas-----------------------------
%Input to create trajectory function to get trajectory.
data = zeros(2,5);
data(1,:) = current;
data(2,:) = target_theta_15;
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
sampling_rate = 300;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
current = data(2,:);
%%---------------------Put Rectangle 15 down-----------------------------------------
%Input to create trajectory function to get trajectory.
data = zeros(2,5);
data(1,:) = current;
data(2,:) = dest_theta_15;
sampling_rate = 300;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
pause(0.01);
GripperAction(openGripperPub);
current = data(2,:);
%%----------------------Move to thetas = 0 --------------------------------
data = zeros(2,5);
data(1,:) = current;
data(2,:) = zeros(1,5);
sampling_rate = 100;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
current = data(2,:);


%% ----------------------Rectangle 14--------------------------------------
SendJointTargetPosition(setPoseArmPub,zeros(1,5));
new_base = GetObjectPosAndOrientation(clientID, vrep, 'base');
SendJointTargetPosition(setPoseArmPub,zeros(1,5));
thetas1 = fast_theta(Rectangle14,base);
current = zeros(1,5);
SendJointTargetPosition(setPoseArmPub,zeros(1,5));
thetas1(1) = thetas1(1) + 0.127;
target_theta_14 = thetas1(1,1:5);
%%-----------------------Move to desired theta1----------------------------
data = zeros(2,5);
data(1,:) = current;
data(2,:) = [target_theta_14(1),0,0,0,0];
sampling_rate = 50;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
current = data(2,:);
%%-----------------------Move to target thetas-----------------------------
%Input to create trajectory function to get trajectory.
data = zeros(2,5);
data(1,:) = current;
data(2,:) = target_theta_14;
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
sampling_rate = 200;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
current = data(2,:);
%%---------------------Put Rectangle 15 down-----------------------------------------
%Input to create trajectory function to get trajectory.
data = zeros(2,5);
data(1,:) = current;
data(2,:) = dest_theta_14;
sampling_rate = 300;
traj = create_trajectory_1a(data,sampling_rate);
Trajectory(traj,setPoseArmPub);
GripperAction(openGripperPub);
current = data(2,:);