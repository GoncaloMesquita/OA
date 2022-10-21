%% clear workspace
clearvars;
close all;

%% Initialization of parameters

load('measurements.mat'); % r1 and r2

global s1
global s2

s1 = [0; -1];
s2 = [1; 5];

epsilon = 1e-6; % Stoping criterarion 
lambda = 1; % Initial lambda

global t
t = 0:0.1:5;

p0 = [-1; 0];
v0 = [0; 1];

n_max_iter = 1000;

x = [p0; v0];

%% LM Algorithm
for k = 1:n_max_iter
   
    % Compute gradient and check stoping criterarion 
    g = grad_f(x(:,k));
    if norm(g)<epsilon
        break;
    end

    % Computes A and b
    A = [];
    b = [];
    for i = 1:2    % Sensor i                    
        A = [A; grad_fi(i,x(:,k))'];
        b = [b; grad_fi(i,x(:,k))'*x(:,k) - fi(i,x(:,k))];   
    end
 
    A = [A; sqrt(lambda)*eye(length(x(:,k)))];
    b = [b; sqrt(lambda)*x(:,k)];
        
    % Compute x(k+1) 
    x_next = A\b;
    
    if f(x_next) < f(x(:,k))
        x(:,k+1) = x_next;
        lambda = 0.7*lambda;
    else
        x(:,k+1) = x(:,k);
        lambda = 2*lambda;
    end
    
    % Data for plots
    cost(k) = f(x(:,k));
    norm_grad_f(k) = norm(g);
end
disp(x(:,k))
%% Plots

figure;
plot(cost, 'linewidth',1);
grid on; grid minor;
xlabel('$k$', 'interpreter', 'latex');
ylabel('cost function $f(x_k)$', 'interpreter', 'latex');

figure;
semilogy(norm_grad_f, 'linewidth',1);
grid on; grid minor;
xlabel('$k$', 'interpreter', 'latex');
ylabel('$||\nabla f(x_k)||_2$', 'interpreter', 'latex');




%% Auxiliar functions
function f = f(x)
    f = 0;
    for i = 1:2   % Sensor i
        f = f+sum(fi(i,x).^2);
    end

end

function fi = fi(i,x)
    global r1;
    global r2;
    global s1;
    global s2;
    global t

    p = x(1:2);
    v = x(3:4); 
    
    switch i
        case 1  % Sensor 1
            fi = vecnorm((p+t.*v)-s1)' - r1;
        case 2 % Sensor 2
            fi = vecnorm((p+t.*v)-s2)' - r2;
        otherwise
    end
end

function grad_f = grad_f(x)

    grad_f = 0;
    
    for i = 1:2   % Sensor i
        grad_f = grad_f + 2*grad_fi(i,x)*fi(i,x);
    end
    

end

function grad_fi = grad_fi(i,x)
    global s1;
    global s2;
    global t;

    p = x(1:2);
    v = x(3:4);    

    switch i
        case 1
            k = ((p+t.*v)-s1);
        case 2
            k = ((p+t.*v)-s2);
        otherwise
    end
    
    grad_fi = [k./vecnorm(k); (k./vecnorm(k)).* t];

end
