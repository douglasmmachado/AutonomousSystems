%----------- STUDENTS FILE ----------------
function x_dot=robot_dynamics(x,u,model)

switch model
    case 'integrator'
        % robot model : simple integrator
        %p_robot = [x;y];
        
        % simple integrator dynamics
        %p_dot = [u1;u2];

        %define space state x = p = [x;y]
        x_dot = u;

    case 'unicycle'
        % robot model : unicycle
        % p = [x;y]
        % theta = heading angle
        % v = speed vector
        % unicycle model
        % p_dot = [v*cos(theta);v*sin(theta)]
        % v_dot = u1
        % theta_dot = u2
        % define x = [x;y;v;theta]
        v = x(3);
        theta = x(4);
        u1 = u(1);
        u2 = u(2);
        x_dot = [v*cos(theta);v*sin(theta);u1;u2]   ;
end
end

