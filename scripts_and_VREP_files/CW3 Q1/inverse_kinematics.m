function thetas = inverse_kinematics(T)
    %Build the robot in peter cox toolbox 
    %Specify DH parameters each link of 4R-planar manipulator
    L1 = Link('d', 0.147, 'a', 0, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
    L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
    L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
    L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
    L5 = Link('d', 0.218, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);
    %Construct the robot
    Youbot = SerialLink([L1, L2, L3, L4, L5]);
    
    thetas = zeros(1,5);
    desired_x = T(1,4);
    desired_y = T(2,4);
    desired_z = T(3,4);
            
    rot_matrix = T(1:3,1:3);
    %Theta 1
    thetas(1) = atan2(desired_y,desired_x);
    T_0_5 = cat(2,rot_matrix,[desired_x;desired_y;desired_z]);
    T_0_5 = cat(1,T_0_5,[0,0,0,1]);
            
    %theta 5
    R31 = rot_matrix(3,1);
    R32 = rot_matrix(3,2);
    thetas(5) = atan2(R32,-R31);
            
    %Transformation matrix from 1 to 4
    T_0_1 = Youbot.A(1,thetas(1));
    T_4_5 = Youbot.A(5,thetas(:));
    T_1_4 = inv(T_0_1) * T_0_5 * inv(T_4_5);
            
    %Theta 3
    r14 = T_1_4(1,4);
    r24 = T_1_4(2,4);
    a2 = 0.155;
    a3 = 0.135;
    thetas(3) = acos((r14^2 + r24^2 - a2^2 - a3^2)/(2*a2*a3));
            
    %Theta 2
    thetas(2) = atan2(-r14,r24) - atan2(a3*sin(thetas(3)),a2+a3*cos(thetas(3)));
            
    %Theta 4 
    r11 = T_1_4(1,1);
    r21 = T_1_4(2,1);
    thetas(4) = atan2(r21,r11) - thetas(2) - thetas(3);
end