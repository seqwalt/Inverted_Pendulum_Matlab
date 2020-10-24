%% --- Inverted Pendulum on a Cart -- Linear Model --- %%

% Adapted from Control Tutorials for Matlab and Simulink:
% https://ctms.engin.umich.edu/CTMS/index.php?aux=Home

%% Parameters
M = .5;    % cart mass
m = 0.2;   % pendulum mass
b = 2;     % drag coefficient
I = 0.006; % pendulum moment of inertia
g = 9.8;   % gravitational acceleration
l = 0.3;   % length to pendulum COM

%% Linearized system dynamics --> x_dot = Ax + Bu, y = Cx + Du = x
p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = eye(4);
D = [0;0;0;0];

%% Control with PD controller K
K = [7 11 -50 -11]; %prorotional and derivative gains
sys = ss(A+B*K,zeros(4,1),C,D);

%% Simulate System Response and Control
t_l = 6; % time length (sec)
t = linspace(0,t_l,t_l*50)'; % 50 Hz sampling rate
x0 = [-2,0,0,0]; % initial conditions [x, dx/dt, th, d(th)/dt]
clear vars y;
y = lsim(sys,zeros(length(t),1),t,x0');
pos   = y(:,1);
d_pos = y(:,2);
ang   = y(:,3);
d_ang = y(:,4);

%% Visualize simulation
visSim_cart_pole([t,pos,ang],0);

%% Plot Output
figure
hold on
plot(t,pos,t,ang)
leg = legend('pos (m)','ang (rad)'); leg.FontSize = 12;
ti = title('Output of Linearized System'); ti.FontSize = 14;
lb = xlabel('Time (sec)'); lb.FontSize = 12;

figure
u = K*y';
plot(t,u)
ti = title('Control Input (N)'); ti.FontSize = 14;
lb = xlabel('Time (sec)'); lb.FontSize = 12;

%% Performance metric
x_int  = trapz(t,abs(pos)); % Area under position minus ref curve
dx_int = trapz(t,abs(d_pos));   % Area under linear velocity curve
a_int  = trapz(t,abs(ang));     % Area under angle curve
da_int = trapz(t,abs(d_ang));   % Area under angular velocity curve

w = [1 0 10 0]; % Here, getting the pendulum to vertical (ang=0) is
                % 10 times more important than getting the cart to its
                % reference position (pos=ref)
X = [x_int; dx_int; a_int; da_int];
C = w*X;             % Weighted cost function
C_max = 37.3947;     % Determined from nonlinear sim w/ pd = [10 7 -70 -3]
P = 1.05*C_max - C;  % Performance metric
fprintf("\nPerformance: %f\n\n",P);
