function force = attractive_field(T,goal)
    current = T(1:3,4);
    delta = goal - current;
    scale_factor = 0.5;
    U_att = 1/2 * scale_factor * delta^2;
end