%---------- Computer lab Autonomous Systems ----------
%-------------- Master MARS - 3A ASI -----------------
%--------------- Lara BRINON ARRANZ ------------------

%----------- STUDENTS SIMULATION FILE ----------------

%% Navigation with artificial potential fields - simple integrator robot

% robot model
% p_robot = [x;y]
% simple integrator dynamics
% p_dot = [u1;u2]

% space state x = p = [x;y]
% x_dot = [u1;u2]

% parameters for the figures
init;
xmin=0;
xmax=10;
ymin=0;
ymax=10;
limits=[xmin, xmax, ymin, ymax];

close all

% scenario
% GOAL
% position of the goal
p_goal=[6;6];

% OBSTACLES
% position of obstacle 1
p_obs=[5;5];
% radius of obstacle 1 : if R_obs=0 the obstacle is a point with no dimensions
R_obs=0;
% position of obstacle 2
p_obs2=p_obs; % p_obs2=p_obs means that there is only one obstacle and not two at the same location
% radius of obstacle 2 : if R_obs2=0 the obstacle is a point with no dimensions
R_obs2=0;

% initial conditions
% robot state initial conditions
x=[2;1];

% simulation parameters
% sampling time
dt=0.01;
% number of iterations
T=300;

% simulation
for k=2:T
    clf(); 
    hold on;
    axis([xmin xmax ymin ymax]); 
    axis square;
    
    % ponderation parameters
    alpha=1;
    beta=1;

    % Gradient computation
    dobs = norm(x(:, k-1) - p_obs);

    % gradient of the repulsive potential function
    grad_Urep= (beta * (x(:, k-1) - p_obs)) / (dobs ^3);
    
    % gradient of the attractive potential function
    grad_Uattr= alpha*2*(x(:, k-1) - p_goal);
    
    % total gradient = desired velocity vector
    grad_U =  ( grad_Uattr - grad_Urep);
    
    %control law
    u(:,k)= - grad_U;
    
        
    % robot state update using its dynamics
    xdot= robot_dynamics(x(:,k-1),u(:,k),'integrator');
    
    x(:,k)= x(:,k-1) + dt*xdot;

    % draw the potential field
    draw_field(p_obs,p_obs2,p_goal,alpha,beta,limits);
    % draw the robot
    plot(x(1,k),x(2,k),'ored','LineWidth',3);
    % draw goal position
    plot(p_goal(1),p_goal(2),'ogreen','LineWidth',4);
    % draw the obstacle
    draw_circular_obstacle(p_obs,R_obs);
    drawnow();
end

%% Figure
figure 
% draw field
draw_field(p_obs,p_obs2,p_goal,alpha,beta,limits);
hold on
axis([xmin xmax ymin ymax]); 
axis square;

% SCENARIO
% plot goal position
plot(p_goal(1),p_goal(2),'ogreen','LineWidth',4);
% plot obstacle position
plot(p_obs(1),p_obs(2),'oblack','LineWidth',3);

% ROBOT
% plot robot's initial state

% plot robot's trajectory

% plot robot's final state


