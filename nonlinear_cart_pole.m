%% --- Inverted Pendulum on a Cart -- Nonlinear Model --- %%

% The function below returns performance P of the system
%function P = nonlinear_cart_pole(pos_kp,pos_kd,ang_kp,ang_kd)

%% Proportional and derivative constants
%pd = [pos_kp pos_kd ang_kp ang_kd]; % <<< uncomment to use function <<<

pd = [7 11 -50 -11]; %[pos_kp pos_kd ang_kp ang_kd] -- human design
%pd = [10 7 -70 -3]; % Poor yet stable parameters
%pd = [5.0186 7.3261 -32.9 -3.71877]; % SafeOpt design
%pd = zeros(1,4);

%% Initial Conditions, reference position, input constraint, time length
x0  = 1; % m
th0 = 0; % rad
v0  = 0; % m/sec
w0  = 0; % rad/sec

ref = -1;        % position reference (m)
ref_max = 1.45; % max dist (m) from reference the controller will consider
                % ref_max = 1.45 originally (found by hand with u_max=2.8)
u_max = 2.8;    % max force (N) controller can apply u_max = 2.8 originally
                % (reasonable for worm-gear dc motor and 5cm radius wheel)
t_l = 6;        % time length in seconds

%% Solve Differential Equations
clear vars y;
[t,y] = ode113(@(t,y) cartpend_dyn(t,y,pd,ref,u_max,ref_max),linspace(0,t_l,t_l*50),[x0; v0; th0; w0]);
pos   = y(:,1);
d_pos = y(:,2);
ang   = y(:,3);
d_ang = y(:,4);

%% Visualize inverted pendulum simulation
visSim_cart_pole([t,pos,ang],ref);

%% Plot Output and Control
% figure
% hold on
% plot(t,pos,t,ang)
% leg = legend('pos (m)','ang (rad)'); leg.FontSize = 12;
% ti = title('Output of Nonlinear System'); ti.FontSize = 14;
% lb = xlabel('Time (sec)'); lb.FontSize = 12;

% "reverse" determine control input from state data
% i.e., this is what the actual controller does in cartpend_dyn.m
y_r = y-[ref,0,0,0];
y_rx = y_r(:,1);
y_rx(y_rx > ref_max) = ref_max;
y_rx(y_rx < -ref_max) = -ref_max;
y_r(:,1) = y_rx;
u = pd*y_r';
u(u>u_max)=u_max;  %Apply constraints to u
u(u<-u_max)=-u_max;
u(abs(ang)>1.57)=0;

% figure
% plot(t,u)
% ti = title('Control Input (N)'); ti.FontSize = 14;
% lb = xlabel('Time (sec)'); lb.FontSize = 12;

%% Performance metric
x_int  = trapz(t,abs(pos-ref)); % Area under position minus ref curve
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
