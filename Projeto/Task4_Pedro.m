clear all

%variables
T = 60; % time steps
y = 0.5;
x_initial = [1;1;0;0];
q1 = importdata('target_1.mat');
q2 = importdata('target_2.mat');
p1 = [0 0.2 0.4 0.6 0.8 1];
p2 = [1 0.8 0.6 0.4 0.2 0];

A = [1 0 0.2 0; 0 1 0 0.2; 0 0 0.8 0; 0 0 0 0.8];
B = [0 0; 0 0; 0.2 0; 0 0.2];
E = [1 0 0 0; 0 1 0 0];

for j=1:length(p1)
    % solve the optimization problem
    TE1 = 0;
    TE2 = 0;
    CE = 0;
    cvx_begin quiet
        variable x(4,T);
        variable u(2,(T-1));

        % Compute TE_1, TE_2
        for i = 1:T
            TE1 = TE1 + max(abs(E*x(:,i) - q1(:,i)));
            TE2 = TE2 + max(abs(E*x(:,i) - q2(:,i)));
        end

        % Compute TE_avg
        TE_avg = p1(j) * TE1 + p2(j) * TE2;

         % Compute CE
        for i = 1:(T-1)
            CE = CE + norm(u(:,i))^2;
        end

        f = TE_avg + y * CE;
        minimize(f);

        % subject t
        x(:,1) == x_initial;
        for i=2:(T)
            x(:,i) == A*x(:,i-1) + B*u(:,i-1);
        end

    cvx_end;

    %plot trajectory of target and vehicle
    figure(j);
    hold on
    plot(x(1,:),x(2,:),'.-k','MarkerSize',10,'LineWidth',1);
    title(sprintf('Trajectories for p1 = %.2f and p2 = %.2f', p1(j),p2(j)));
    plot(q1(1,:),q1(2,:),'.-r','MarkerSize',10,'LineWidth',1);
    plot(q2(1,:),q2(2,:),'.-m','MarkerSize',10,'LineWidth',1);
    xlim([-1.5 1.5])
    ylim([-1.5 1.5])
    grid(gca,'minor')
    grid on
    saveas(gcf,sprintf('Trajectories for p1 = %.2f and p2 = %.2f .png', p1(j),p2(j)))
end


