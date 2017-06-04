function theta = theta_finder(clientID,vrep,target,option)
    L1 = Link('d', 0.1013, 'a', 0, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
    L2 = Link('d', 0, 'a', 0.1549, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
    L3 = Link('d', 0, 'a', 0.1349, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
    L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
    base = GetObjectPosAndOrientation(clientID, vrep, 'youBotArmJoint0');
    
    t_position = target.position;
    b_position = base.position;
    
    %Displacement and theta
    disp = t_position - b_position;
    theta1 = -(-pi/2 - atan2(disp(2),disp(1))); 
    
    %Rectangle 15 is a special case, the theta1 can not be reached.
    if ~strcmp(option,'R15')
        theta1 = -(-pi/2 - atan2(disp(2),disp(1))) + 0.02;
    end
    
    %Record all the possible link 5 length
    record = ones(470,2);
    record = record .* 10000000;
    
    for i = 1:470
        try
        d = 0.171 + i/10000;
        L5 = Link('d', d, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);
        Youbot = SerialLink([L1, L2, L3, L4, L5]);
        
        R = eul2rotm([theta1 0 0]);
        T = [R(1,:) disp(2);R(2,:) -disp(1);R(3,:) disp(3);0 0 0 1];
        target_theta = Youbot.ikine(T,[theta1 0 0 0 0],[1,1,1,0,0,0]);
        [T_end,~] = Youbot.fkine(target_theta);
        
        distance = [t_position(1) + T_end(2,4) - b_position(1),t_position(2) - T_end(1,4) - b_position(2),t_position(3) - T_end(3,4) - b_position(3)];
        record(i,1) = d;
        record(i,2) = norm(distance);
        catch
        end
    end
    
    counter = 1;
    theta = zeros(1,6);
    %theta = [-1.1967,1.0578,0.7457,0.3320,0,0];
    for j = 1:470
        if record(j,2) == 0 
            d_final = 0.171 + j/10000;
            L5 = Link('d', d_final, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);
            Youbot = SerialLink([L1, L2, L3, L4, L5]);
            R = eul2rotm([theta1 0 0]);
            T = [R(1,:) disp(2);R(2,:) -disp(1);R(3,:) disp(3);0 0 0 1];
            target_theta = Youbot.ikine(T,[theta1 0 0 0 0],[1,1,1,0,0,0]);
            theta = vertcat(theta,zeros(1,6));
            theta(counter,1:5) = target_theta;
            theta(counter,6) = d_final;
            counter = counter + 1;
        end
    end
 end