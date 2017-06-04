function [joint_profile, velocity_profile] = create_trajectory_1c(data,sampling_rate)   
    n = floor((sampling_rate - 2)/3);
    sampling_rate = 3*n + 3;
    
    %Initialization
    Num_pose = size(data,3);
    velocity_profile = zeros(sampling_rate*(Num_pose-1),5);
    joint_profile = zeros(sampling_rate*(Num_pose-1),5);
   
    %For each period
    for station = 1:(Num_pose-1)
  
        d_0 = data(:,:,station);
        d_1 = data(:,:,station+1);
        theta_0 = inverse_kinematics(d_0);
        theta_1 = inverse_kinematics(d_1);
        
        delta_theta = theta_1 - theta_0;
        velocity_max = (delta_theta ./ 2);        
        accelerate = velocity_max;
        decelerate = -accelerate;
        delta_t = 1/(n+1);
        
        section_start = sampling_rate * (station - 1) + 1;
        joint_profile(section_start,:) = theta_0;
        
        %Accelerate period
        for a_index = 1:n
            t = delta_t * a_index;
            velocity = t * accelerate;
            velocity_profile(section_start + a_index,:) = velocity;
            position = theta_0 + 1/2 * t * velocity;
            joint_profile(section_start + a_index,:) = position;
        end
        
        velocity_profile(section_start + n + 1,:) = velocity_max;
        joint_profile(section_start + n + 1,:) = theta_0 + 1/2 * (n+1) * delta_t * velocity_max;
        
        %Constant speed period
        for c_index = 1:n
            i = c_index + 1 + n;
            t = delta_t * (i);
            velocity_profile(i + section_start,:) = velocity_max;
            position = theta_0 + 1/2 * (t + c_index*delta_t) * velocity_max;
            joint_profile(i + section_start,:) = position;
        end
        
        velocity_profile(section_start + 2*n + 2,:) = velocity_max;
        old = 1/2 * 3 * velocity_max;
        joint_profile(section_start + 2*n + 2,:) = theta_0 + old;
        
        %Decelerate period
        for d_index = 1:n
            i = d_index + 2 + 2*n;
            velocity = velocity_max + d_index * delta_t * decelerate;
            velocity_profile(section_start + i,:) = velocity;
            new = 1/2 * (velocity + velocity_max) * (d_index * delta_t); 
            position = old + new;
            joint_profile(section_start + i,:) = theta_0 + position;
        end
    end
end