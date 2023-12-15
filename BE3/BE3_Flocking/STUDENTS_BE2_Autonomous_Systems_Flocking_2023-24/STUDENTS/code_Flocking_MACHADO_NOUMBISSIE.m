%---------- Computer lab Autonomous Systems ----------
%-------------- Master MARS - 3A ASI -----------------
%--------------- Lara BRINON ARRANZ ------------------

% ----------------- STUDENTS FILE --------------------
%% Flocking for double integrator robots

% robot model
% pi = [xi;yi]
% vi = [vix;viy]
% double integrator
% pi_dot = vi
% vi_dot = ui

init;

% simulation parameters
%number of robots
N=20;

% Adjacency matrix
A=ones(N);
A = A.*-(eye(height(A))-1); % Making the diagonal zero as there is no connection within nodes

% Degree matrix
degree = diag(sum(A)); % Degree matrix

% sampling time
dt=1/(2*N);

% iterations
T=200;


% Variables to be used in the simulation
% vector of all robot states P=[p1;p2;...;pN] 
% X=[x1;x2;...;xN] Y=[y1;y2;...;yN]
% We keep in memory the values at each iteration k
X=zeros(N,T);
Y=zeros(N,T);

% random robot initial states
P0=20*randn(N,2);
X(:,1)=P0(:,1);
Y(:,1)=P0(:,2);

V0=5+10*randn(N,2);
Vx=V0(:,1);
Vy=V0(:,2);


% vector of all control inputs U=[u1;u2;...;uN]
% Ux=[ux1;ux2;...;uxN] Uy=[uy1;uy2;...;uyN]
% we update the value of U at each iteration k in the same variable
% initial conditions for control inputs
Ux=zeros(N,1);
Uy=zeros(N,1);


% SIMULATION
for k=2:T
    clf();
    axis([-60,60,-60,60]);
    axis square; hold on

    % control parameters
    alpha=2;
    beta=1;
    gamma=1;

    for i=1:N
        
        
        % alignment
        for j =1:N
            Ux(i) = - alpha * A(i,j) * (Vx(i,k-1) - Vx(j,k-1));
            Uy(i) = - alpha *  A(i,j) * (Vy(i,k-1) - Vy(j,k-1));
        end

        % cohesion
        Ux(i)= Ux(i) - beta * X(i, k-1);
        Uy(i)= Uy(i) - beta * Y(i, k-1);
        
        % separation
        Ux(i)= Ux(i) - gamma * X(i, k-1);
        Uy(i)= Uy(i) - gamma * Y(i, k-1);
        
        
        % update the state of robot i using its dynamics
        Vx(i,k)=Vx(i,k-1) + Ux(i).*dt;
        Vy(i,k)=Vy(i,k-1) + Uy(i).*dt;

        X(i,k)=X(i,k-1) + Vx(i,k).*dt;
        Y(i,k)=Y(i,k-1) + Vy(i,k).*dt;

        % draw the robots as circles
        plot(X(i,k),Y(i,k),'oblack','LineWidth',3)
        % draw the robots' velocities as arrows
        quiver(X(i,k),Y(i,k),Vx(i,k),Vy(i,k),'LineWidth',2)
    
    end
    drawnow();
end

%% Figure
figure 
% plot the robots' initial positions in red
plot(X(:,1),Y(:,1),'ored','LineWidth',3);
hold on
axis square;

% draw the robots' initial velocities as arrows
for i=1:N
    quiver(X(i,1),Y(i,1),Vx(i,1),Vy(i,1),'r','LineWidth',1)
end

% plot the robots' trajectories in black
plot(X(:,2:T-1),Y(:,2:T-1),'black*','MarkerSize',1);
hold on

% plot the robots' final positions in blue
plot(X(:,T),Y(:,T),'oblue','LineWidth',3);
hold on


% draw the robots' final velocities as arrows
for i=1:N
    quiver(X(i,T),Y(i,T),Vx(i,T),Vy(i,T),'blue','LineWidth',1)
end
hold off

%%

figure
% plot the robots' velocities over time
subplot(2,1,1);
plot(Vx(:,1),Vy(:,1),'ored','LineWidth',1)

subplot(2,1,2)
plot(Vx(:,T),Vy(:,T),'oblue','LineWidth',1)
%%
figure
% plot the robots' positions over time
subplot(2,1,1);
plot(X(:,1),Y(:,1),'ored','LineWidth',1)

subplot(2,1,2);
plot(X(:,T),Y(:,T),'oblue','LineWidth',1)