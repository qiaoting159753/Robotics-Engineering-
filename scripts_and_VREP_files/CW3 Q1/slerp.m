function T = slerp(rot_0,rot_1,sampling_rate)
    T = zeros(3,3,sampling_rate + 1);
    T(:,:,1) = rot_0;
    %Rotation to quatanine
    Q_0=rotm2quat(rot_0);
    Q_1=rotm2quat(rot_1);
    %normalize
    Q_0 = Q_0/norm(Q_0);
    Q_1 = Q_1/norm(Q_1);
    
    %Dot product
    D = dot(Q_0,Q_1);
    dot_threshold = 0.95;
    
    %If two quatanine are too close, using the linear interpolation
    if D>dot_threshold 
        for i = 2:(sampling_rate+1)
            Q = Q_0 + (Q_1 - Q_0)/(sampling_rate + 1) * (i-1);
            Q = Q/norm(Q);
            T(:,:,i) = quat2rotm(Q);
        end
        return
    end
    
    if D < 0
        Q_0 = -Q_0;
        D = -D;
    end
    
    Q_2 = Q_1 - Q_0 * D;
    Q_2 = Q_2/norm(Q_2);
    theta_0 = acos(D);
    for i = 2:(sampling_rate+1)
        theta = (i-1)*theta_0/(sampling_rate+1);
        Q = Q_0*cos(theta) + Q_2*sin(theta);
        Q = Q/norm(Q);
        T(:,:,i) = quat2rotm(Q);
    end
end