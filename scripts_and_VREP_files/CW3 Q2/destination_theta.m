function theta = destination_theta(clientID,vrep,destination,option)
    L1 = Link('d', 0.1013, 'a', 0, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
    L2 = Link('d', 0, 'a', 0.1549, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
    L3 = Link('d', 0, 'a', 0.1349, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
    L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
    
    base = GetObjectPosAndOrientation(clientID, vrep, 'youBotArmJoint0');
    
    offset = 0.02;
    if strcmp(option,'R15')
        offset = 0.03;
    end
    %0.065
    destination(1) = destination(1)-offset-0.007;
    destination(2) = destination(2)-offset;
    b_position = base.position;
    disp = destination - base.position;
    theta1 = -(-pi/2 - atan2(disp(2),disp(1)));

    
    length = 0.2;
    record = ones(101,2);
    record = record .* 10000000;
    
    counter1 = 1;
    for i = 1:300
        try
            d_final = length - i/10000;
            L5 = Link('d', d_final, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);
            Youbot = SerialLink([L1, L2, L3, L4, L5]);
            
            R = eul2rotm([theta1 0 0]);
            T = [R(1,:) disp(2);R(2,:) -disp(1);R(3,:) disp(3);0 0 0 1];
            target_theta = Youbot.ikine(T,[theta1 0 0 0 0],[1,1,1,0,0,0]);
            [T_end,~] = Youbot.fkine(target_theta);
            distance = [destination(1) + T_end(2,4) - b_position(1),destination(2) - T_end(1,4) - b_position(2),destination(3) - T_end(3,4) - b_position(3)];
            record(counter1,1) = d_final;
            record(counter1,2) = norm(distance);
            counter1 = counter1 + 1;
        catch
        end
    end
    
    
    destination(1) = destination(1) + offset;
    destination(2) = destination(2) + offset;
    disp2 = destination - base.position;
    theta2 = -(-pi/2 - atan2(disp2(2),disp2(1)));
    
    counter = 1;
    theta = zeros(1,6);
    for j = 1:101
        if record(j,2) == 0 
            d_final = length + j/10000;
            L5 = Link('d', d_final, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);
            Youbot = SerialLink([L1, L2, L3, L4, L5]);
            R = eul2rotm([theta1 0 0]);
            T = [R(1,:) disp(2);R(2,:) -disp(1);R(3,:) disp(3);0 0 0 1];
            target_theta = Youbot.ikine(T,[theta1 pi/9 pi/9 pi/6 0],[1,1,1,0,0,0]);
            theta = vertcat(theta,zeros(1,6));
            theta(counter,1:5) = target_theta;
            theta(counter,6) = d_final;
            theta(counter,1) = theta2;
            counter = counter + 1;
        end
    end
end