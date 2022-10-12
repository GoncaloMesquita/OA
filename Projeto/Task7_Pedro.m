clear all

%variables
T = 60; %time steps
T_midway = 35;
K = 2;  %number of trajectorys
y = 0.05;%weigth
x_initial = [1;1;0;0];
q1 = importdata('target_1.mat');
q2 = importdata('target_2.mat');
p = [0.6 0.4]; %probability of trajectories

A = [1 0 0.2 0; 0 1 0 0.2; 0 0 0.8 0; 0 0 0 0.8];
B = [0 0; 0 0; 0.2 0; 0 0.2];
E = [1 0 0 0; 0 1 0 0];

% solve the optimization problem
TE1 = 0;
TE2 = 0;
CE1 = 0;
CE2 = 0;

cvx_begin quiet
    variable x1(4,T);
    variable x2(4,T);
    variable u1(2,(T-1));
    variable u2(2,(T-1));

    % Compute 
    for i = 1:T
        TE1 = TE1 + max(abs(E*x1(:,i) - q1(:,i)));
        TE2 = TE2 + max(abs(E*x2(:,i) - q2(:,i)));
    end

     % Compute CE
    for i = 1:(T-1)
        CE1 = CE1 + norm(u1(:,i))^2;
        CE2 = CE2 + norm(u2(:,i))^2;
    end

    f = (p(1) * (TE1 + y * CE1)) + (p(2) * (TE2 + y * CE2));
    minimize(f);

    % subject to
    x1(:,1) == x_initial;
    for i = 2:(T)
        x1(:,i) == A*x1(:,i-1) + B*u1(:,i-1);
    end
    
    x2(:,1) == x_initial;
    for i = 2:(T)
        x2(:,i) == A*x2(:,i-1) + B*u2(:,i-1);
    end
    
    for i = 1:(T_midway - 1)
        u1(:,i) == u2(:,i);
    end
   
cvx_end;

%plot trajectory of target and vehicle
figure;
hold on
plot(x1(1,:),x1(2,:),'.-y','MarkerSize',10,'LineWidth',1);
plot(x2(1,:),x2(2,:),'.-y','MarkerSize',10,'LineWidth',1);
title(sprintf('Trajectories for p1 = %.2f and p2 = %.2f', p(1),p(2)));
plot(q1(1,:),q1(2,:),'.-r','MarkerSize',10,'LineWidth',1);
plot(q2(1,:),q2(2,:),'.-m','MarkerSize',10,'LineWidth',1);
xlim([-1.5 1.5])
ylim([-1.5 1.5])
grid(gca,'minor')
grid on
%saveas(gcf,sprintf('Trajectories for p1 = %.2f and p2 = %.2f .png', p(1),p(2)))



