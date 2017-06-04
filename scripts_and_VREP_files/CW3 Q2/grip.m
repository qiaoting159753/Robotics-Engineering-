moving = 0;
for k = 1:100
    time = (k-1)*0.05;
    %Open to be ready to grip
    if moving == 0
        GripperAction(openGripperPub);
        moving = 1;
    end
    %Grip
    if time == 0.5
        GripperAction(closeGripperPub);
    end
    pause(0.02);
end