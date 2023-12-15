%---------- Computer lab Autonomous Systems ----------
%-------------- Master MARS - 3A ASI -----------------
%--------------- Lara BRINON ARRANZ ------------------

%----------- STUDENTS SIMULATION FILE ----------------

%% Navigation with artificial potential fields - non-holonomic robot

% robot model
% p = [x;y]
% theta = heading angle
% v = speed vector
% unicycle car model
% p_dot = [v*cos(theta);v*sin(theta)]
% v_dot = u1
% theta_dot = u2
% define x = [x;y;v;theta]
% x_dot = [v*cos(theta);v*sin(theta);u1;u2]

% parameters for the figures
init;
xmin=0;
xmax=10;
ymin=0;
ymax=10;
limits=[xmin, xmax, ymin, ymax];

% scenario
% GOAL
% position of the goal
p_goal=[6;6];

% OBSTACLES
% position of obstacle 1
p_obs=[4;4];
% radius of obstacle 1 : if R_obs=0 the obstacle is a point with no dimensions
R_obs=0.4;
% position of obstacle 2
p_obs2=[6;4]; % p_obs2=p_obs means that there is only one obstacle and not two at the same location
% radius of obstacle 2 : if R_obs2=0 the obstacle is a point with no dimensions
R_obs2=0.1;

% initial conditions
% robot state initial conditions
x=[2;1;1;pi/4];

% simulation parameters
% sampling time
dt=0.005;
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
    dobs1 = distance_obs(x(1:2, k-1),p_obs,R_obs);
    dobs2 = distance_obs(x(1:2, k-1),p_obs2,R_obs2);

    % gradient of the repulsive potential function
    grad_Urep= (beta * ((dobs1 / norm(dobs1)^3) + (dobs2 / norm(dobs2)^3)));
    
    % gradient of the attractive potential function
    grad_Uattr= alpha*2*(x(1:2, k-1) - p_goal);
    
    % total gradient = desired velocity vector
    grad_U =  ( grad_Uattr - grad_Urep);
    
    % reference velocity vector
    ref_vel_vec=[0;0];

    % desired speed vector
    vref=norm(grad_U);
    % desired heading angle
    thetaref=atan2(- grad_U(2) , - grad_U(1));

    v = x(3, k-1);
    theta = x(4, k-1);

    %control law
    %control parameters
    K1=1;
    K2=10;
    %control input
    u(:,k)=[-K1*(v - vref);-K2*atan(tan((theta - thetaref)/2))];
          
    % robot state update using its dynamics
    xdot=robot_dynamics(x(:,k-1), u(:,k), 'unicycle');
    x(:,k)= x(:,k-1) + dt*xdot;

    % draw the potential field
    draw_field(p_obs,p_obs2,p_goal,alpha,beta,limits);
    % draw the robot
    draw_robot(x([1,2,4],k),'red',0.1);
    % draw goal position
    plot(p_goal(1),p_goal(2),'ogreen','LineWidth',4);
    % draw the obstacle(s)
    draw_circular_obstacle(p_obs,R_obs);
    draw_circular_obstacle(p_obs2,R_obs2);
    drawnow();
end

%% Figures
figure 
draw_field(p_obs,p_obs2,p_goal,alpha,beta,limits);
hold on
axis([xmin xmax ymin ymax]); 
axis square;

% SCENARIO
% plot goal position
plot(p_goal(1),p_goal(2),'ogreen','LineWidth',4);
% plot the obstacle(s)
draw_circular_obstacle(p_obs,R_obs);

% ROBOT
% plot robot's initial state

% plot robot's trajectory

% plot robot's final state

