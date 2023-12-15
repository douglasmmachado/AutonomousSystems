%---------- Computer lab Autonomous Systems ----------
%-------------- Master MARS - 3A ASI -----------------
%--------------- Lara BRINON ARRANZ ------------------

% ---------------- STUDENTS FILE ---------------------
%% Consensus for simple integrator robots

% position of robot i (2D)
% pi = [xi;yi]
% robot model: simple integrator
% pi_dot = ui = [uxi;uyi]

init;
close all

% Select the question to be simulated
question = 'Question2';
switch question
    case 'Question1' % Question 1: all-to-all communication
        % number of robots
        N=100; % N should be =< 100
        % Adjacency matrix
        A=ones(N);
        A = A.*-(eye(height(A))-1); % Making the diagonal zero as there is no connection within nodes

        % Degree matrix
        degree = diag(sum(A)); % Degree matrix
        % Laplacian matrix
        L= degree - A;

    case 'Question2' % Question 2: different communication graphs
        % number of robots
        N=4;
        % Select the graph of the network to be simulated       
        graph_type = 'G7';
        switch graph_type
            case 'G1'
                A=ones(N);
                A = A.*-(eye(height(A))-1);
                D = diag(sum(A));
                L = D - A;
            case 'G2'
                A = [0 1 0 1; 1 0 1 1; 0 1 0 1; 1 1 1 0];
                D = diag(sum(A));
                L=D - A;
            case 'G3'
                A = [0 1 0 1; 1 0 1 0; 0 1 0 1; 1 0 1 0];
                D = diag(sum(A));
                L=D - A;
            case 'G4'
                A = [0 0 1 0; 0 0 1 0; 1 1 0 1; 0 0 1 0];
                D = diag(sum(A));
                L = D - A;
            case 'G5'
                A = [0 0 0 1; 1 0 0 0; 0 1 0 0; 0 1 1 0];
                D = diag(sum(A5, 2));
                L = D - A;
             case 'G6'
                A = [0 0 0 0; 1 0 1 0; 0 0 0 1; 0 0 0 0];
                D = diag(sum(A5, 2));
                L = D - A;
            case 'G7'
                A = [0 0 0 0; 1 0 1 0; 0 0 0 0; 0 0 1 0];
                D = diag(sum(A5, 2));
                L = D - A;


        end
end


% simulation parameters
switch question
    case 'Question1' % Question 1: all-to-all communication
        % sampling time
        dt=1/(2*100); % max(N)=100
        % iterations
        T=200;
    case 'Question2' % Question 2: different communication graphs
        % sampling time
        dt=1/(2*N);
        % iterations
        T=100;
end

% Variables to be used in the simulation
% vector of all robot states P=[p1;p2;...;pN] 
% X=[x1;x2;...;xN] Y=[y1;y2;...;yN]
% We keep in memory the values at each iteration k
X=zeros(N,T);
Y=zeros(N,T);

% initial conditions (k=1)
switch question
    case 'Question1' % Question 1: all-to-all communication
        % random initial conditions for Question 1
        P0=25*randn(N,2);
        X(:,1)=P0(:,1); 
        Y(:,1)=P0(:,2);
    case 'Question2' % Question 2: different communication graphs
        % initial conditions for Question 2
        X(:,1)=[42;-5;-54;-21];
        Y(:,1)=[34;-26;14;3];
end

% vector of all control inputs U=[u1;u2;...;uN]
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
    alpha=1; % consensus 
    beta=.05;  % repulsive potential

    for i=1:N
        % control law for robot i
        Ux(i)= beta * X(i, k-1);
        Uy(i)= beta * Y(i, k-1);
        switch question
            case 'Question1' % Question 1: all-to-all communication
                % code your consensus algorithm here 
                
                %for j = 1:N   
                %    Ux(i) = Ux(i) - ( A(i,j) * (X(i,k-1) - X(j,k-1)));
                %    Uy(i) = Uy(i) - ( A(i,j) * (Y(i,k-1) - Y(j,k-1)));
                %end
            

                % Question 1.5
                % consensus with Laplacian (all-to-all communication)
                % code your consensus with Laplacian here
                Ux(i) = Ux(i) - alpha * L(i,:) * (X(:,k-1) - X(i,k-1));
                Uy(i) = Uy(i) - alpha * L(i,:) * (Y(:,k-1) - Y(i,k-1));
          
            case 'Question2' % Question 2: consensus with Laplacian
                % code your consensus algorithm here
                Ux(i) = Ux(i) - alpha * L(i,:) * (X(:,k-1) - X(i,k-1));
                Uy(i) = Uy(i) - alpha * L(i,:) * (Y(:,k-1) - Y(i,k-1));

        end
       
        % update the position of robot i using its dynamics
        X(i,k)=X(i,k-1) + Ux(i).*dt;
        Y(i,k)=Y(i,k-1) + Uy(i).*dt;
        % draw the robots as circles
        plot(X(i,k),Y(i,k),'oblack','LineWidth',3)
    end
    drawnow();
end

%%

X1 = X;
Y1 = Y;
%%

X2 = X;
Y2 = Y;
%%

X3 = X;
Y3 = Y;
%%
X4 = X;
Y4 = Y;

%%

figure
subplot(3,1,1)
% plot the robots' initial states
p1 = plot(X1(:,1),Y1(:,1),'oblack','LineWidth',3);
hold on

% plot the robots' trajectories
p2 = plot(X1(:,2:T-1),Y1(:,2:T-1),'blue*','MarkerSize',1);
hold on

% plot the robots' final states
p3 = plot(X1(:,T),Y1(:,T),'ored','LineWidth',3);

hold off

legend([p1(1),p2(1),p3(1)],{'Initial state', 'Trajectory', 'Final state'})
title('G5')
xlabel('X')
ylabel('Y')
grid on


subplot(3,1,2)
% plot the robots' initial states
p1 = plot(X2(:,1),Y2(:,1),'oblack','LineWidth',3);
hold on

% plot the robots' trajectories
p2 = plot(X2(:,2:T-1),Y2(:,2:T-1),'blue*','MarkerSize',1);
hold on

% plot the robots' final states
p3 = plot(X2(:,T),Y2(:,T),'ored','LineWidth',3);

hold off

legend([p1(1),p2(1),p3(1)],{'Initial state', 'Trajectory', 'Final state'})
title('G6')
xlabel('X')
ylabel('Y')
grid on

subplot(3,1,3)
% plot the robots' initial states
p1 = plot(X3(:,1),Y3(:,1),'oblack','LineWidth',3);
hold on

% plot the robots' trajectories
p2 = plot(X3(:,2:T-1),Y3(:,2:T-1),'blue*','MarkerSize',1);
hold on

% plot the robots' final states
p3 = plot(X3(:,T),Y3(:,T),'ored','LineWidth',3);

hold off

legend([p1(1),p2(1),p3(1)],{'Initial state', 'Trajectory', 'Final state'})
title('G7')
xlabel('X')
ylabel('Y')
grid on

%% Figures
% 1st figure with the initial and final positions 
% as well as the 2D trajectories

figure
p1 = plot(X(:,1),Y(:,1),'oblack','LineWidth',3) % initial position
title('Dynamics of all-to-all communication network with N=10 robots')
xlabel('X')
ylabel('Y')
grid on
hold on

p2 = plot(X(:,2:2:T-1),Y(:,2:2:T-1),'blue*','MarkerSize',1) % Trajectory
hold on 

p3 = plot(X(:,T),Y(:,T),'ored','LineWidth',3) % Final position
hold off
legend([p1(1),p2(1),p3(1)],{'Initial state', 'Trajectory', 'Final state'})


figure 
% plot the robots' initial states
p1 = plot(X(:,1),Y(:,1),'oblack','LineWidth',3);
hold on

% plot the robots' trajectories
p2 = plot(X(:,2:10:T-1),Y(:,2:10:T-1),'oblue','LineWidth',3);
hold on

% plot the robots' final states
p3 = plot(X(:,T),Y(:,T),'ored','LineWidth',3);

hold off

legend([p1(1),p2(1),p3(1)],{'Initial state', 'Trajectory', 'Final state'})
title('Robots position through time')
xlabel('X')
ylabel('Y')
grid on
