function [joint_profile, velocity_profile] = create_trajectory_1b(data,sampling_rate)
    %How many points we need to pass
    [M, N, O] = size(data);
    velocity_profile = zeros(sampling_rate*(O-1),5);
    joint_profile = zeros(sampling_rate*(O-1),5);

    %For each period
    for station = 1:(O-1)
        d_0 = data(:,:,station);
        d_1 = data(:,:,station+1);
        delta = (d_1 - d_0)/(sampling_rate);
        
        %[x,y,z,rx,ry,rz]
        desired_tra = zeros(sampling_rate,5);
        desired_tra(1,:) = inverse_kinematics(d_0);
        
        %Slerp pose interpolation
        rot_0 = d_0(1:3,1:3);
        rot_1 = d_1(1:3,1:3);
        T = slerp(rot_0,rot_1,sampling_rate - 1);
        
        for via = 1:sampling_rate
            %Transformation matrix for each via points
            transform_matrix = d_0 + delta * (via-1);
            transform_matrix(1:3,1:3) = T(:,:,via);
            theta = inverse_kinematics(transform_matrix);
            desired_tra(via,:) = theta;
        end
        
        start = (station-1) * sampling_rate + 1;
        finish = start + sampling_rate - 1;
        joint_profile(start:finish,:) = desired_tra;
    end
end