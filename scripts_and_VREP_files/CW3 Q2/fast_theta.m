function theta = fast_theta(object,base)
    %From the scene the L1 length is about 0.1025
    L1 = Link('d', 0.1025, 'a', 0, 'alpha', pi/2, 'qlim', [deg2rad(-180) deg2rad(180)]);
    L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(65)]);
    L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(110)]);
    L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
    L5 = Link('d', 0.218, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);
    Youbot = SerialLink([L1, L2, L3, L4, L5]);
    
    b_position = base.position;
    o_position = object.position;
    
    %Displacement
    d = o_position - b_position;
    %Work out the theta first
    theta_1 = -(-pi/2 - atan2(d(2),d(1)));
    
    o_position = o_position - (eul2rotm([theta_1 0 0])*[0.02;0;0])';
    d = o_position - b_position;
    theta_1 = -(-pi/2 - atan2(d(2),d(1)));
    
    %Average the Offset to both x and y axis.
    d(1) = 0.043*d(1)/norm(d(1:2)) + d(1);
    d(2) = 0.043*d(2)/norm(d(1:2)) + d(2);
    
    %Since the Youbot coordinate frame Rotated twice at Link1 and Link5
    R = eul2rotm([theta_1 0 0])*eul2rotm([0 0 pi/2])*eul2rotm([0 0 pi/2]);
    %Compose the Transition matrix.The x and y are opposite. 
    T = [R(1,:) d(2);R(2,:) -d(1);R(3,:) d(3);0 0 0 1];
    %Inverse Kinimatics.
    theta = Youbot.ikine(T,[theta1 pi/9 pi/6 pi/6 0],[1,1,1,0,0,0]);
    %theta(1) = theta1 - pi;
end
    