%% clear workspace
clearvars;
close all;

%% Define the simulation parameters
q = importdata('target_1.mat');
A = [1 0 0.2 0; 0 1 0 0.2; 0 0 0.8 0; 0 0 0 0.8];
B = [0 0; 0 0; 0.2 0; 0 0.2];
E = [1 0 0 0; 0 1 0 0];
lambdas = [10 5 1 0.5 0.1 0.05 0.01 0.005 0.001];

T = size(q,2);
x_initial = [1;1;0;0];
Lambda_values = cell(1,size(lambdas,2));

%% Run simulation
for i = 1:size(lambdas,2)
    cvx_begin quiet
    
        variable x(4,T);
        variable u(2,T-1);
        
        % Compute TE
        TE_aux{i} = 0;
        for k = 1:T
          TE_aux{i} = TE_aux{i} + norm(E*x(:,k) - q(:,k),inf);
        end

        % Compute CE
        CE_aux{i} = 0;
        for k = 1:T-1
            CE_aux{i} = CE_aux{i} + sum_square(u(:,k));
        end

        minimize(TE_aux{i} + lambdas(i)*CE_aux{i});
        
        % subject to
        x(:,1) == x_initial;
        for t = 1:(T-1)
            x(:,t+1) == A*x(:,t) + B*u(:,t);  
        end
 
    cvx_end;
    TE = cell2mat(TE_aux);
    CE = cell2mat(CE_aux);
    Lambda_values{i} = sprintf('\\lambda_{%.f} = %.3f', i, lambdas(i));
    
    % Position Plots
    figure;
    plot(x(1,:),x(2,:),'-r.','DisplayName',sprintf('Position for lambda = %.3f', lambdas(i)))
    hold on
    plot(q(1,:),q(2,:),'-b.','DisplayName','Target position')
    legend
    axis([-1.5 1.5 -1.5 1.5])    
end

% Lambda Plots
figure;
scatter(TE,CE,'b.')
text(TE+0.3,CE+0.3,Lambda_values,'Fontsize', 8,'HorizontalAlignment', 'left','VerticalAlignment', 'bottom');
xlabel('TE')
ylabel('CE')
grid on
grid minor
