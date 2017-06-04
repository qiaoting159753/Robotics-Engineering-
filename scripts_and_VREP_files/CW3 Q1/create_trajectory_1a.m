function [joint_profile, velocity_profile] = create_trajectory_1a(data,sampling_rate)
    [M, N] = size(data);
    %Size is 5 * sampling_rate
    joint_profile = zeros(sampling_rate*(M-1)+1,N);
    velocity_profile = zeros(sampling_rate*(M-1),N);
    
    %1,2,3,4
    %For 5 time sampling
    for index = 1:(M-1)
        %The [0,1,2,3,4] * sampling_rate + 1 is the data itself
        joint_profile((index-1)*sampling_rate + 1,:) = data(index,:);
        
        %Calculate the coefficient 
        %t_i = 0;
        t_i = 0 + 3*(index -1);
        %t_f = 3,6,9,12
        t_f = 3 + t_i;
    
        A = [1,t_i,(t_i^2),(t_i^3)  ,(t_i^4)   ,(t_i^5);
             0,1  ,2*t_i  ,3*(t_i^2),4*(t_i^3) ,5*(t_i^4);
             0,0  ,2      ,6*t_i    ,12*(t_i^2),20*(t_i^3);
             1,t_f,(t_f^2),(t_f^3)  ,(t_f^4)   ,(t_f^5);
             0,1  ,2*t_f  ,3*(t_f^2),4*(t_f^3) ,5*(t_f^4);
             0,0  ,2      ,6*t_f    ,12*(t_f^2),20*(t_f^3)];
       
        %For each joint
        for joint_index = 1:N
            q_i = data(index,joint_index);
            q_f = data(index+1,joint_index);
            B = [q_i;0;0;q_f;0;0];
            coefficient = inv(A) * B;
            %For each sampling time
            for i = 1:(sampling_rate-1)
                t = t_i + (3/sampling_rate) * (i);
                polynomial = [1,t,t^2,t^3,t^4,t^5];
                degree = [1;2;3;4;5];
                derivative_coe = degree .* coefficient(2:6);
                velocity_profile((index-1)*sampling_rate + i + 1,joint_index) = polynomial(1:5) * derivative_coe;
                joint_profile((index-1)*sampling_rate + i + 1,joint_index) = polynomial * coefficient; 
            end
        end
    end
    joint_profile(sampling_rate*(M-1)+1,:) = data(M,:);
end