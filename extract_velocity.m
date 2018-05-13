function [x_dot y_dot theta_dot] = extract_velocity(vel_right, vel_left, angle)
    
    r = 0.038;
    L = 0.3175;
    x_dot = (r/2)*(vel_right+vel_left)*cos(angle);
    y_dot = (r/2)*(vel_right+vel_left)*sin(angle);
    theta_dot = (r/L)*(vel_right-vel_left);
    
end