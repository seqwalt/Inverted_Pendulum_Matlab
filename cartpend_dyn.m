% --- Nonlinear dynamics of the inverted pendulum cart --- %

function dydt = cartpend_dyn(t,y,pd,xref,umax,refmax)

M = .5;     % cart mass
m = 0.2;    % pendulum mass
b = 2;      % drag coefficient
I = 0.006;  % pendulum moment of inertia
g = 9.8;    % gravitational acceleration
l = 0.3;    % length to pendulum COM

x = y(1);   % position
dx = y(2);  % dx/dt
th = y(3);  % theta = 0 means upright pendulum
dth = y(4); % d(th)/dt

% PD controller
delta_x = x - xref;
delta_x(abs(delta_x) > refmax) = sign(delta_x)*refmax;
u = pd(1)*(delta_x) + pd(2)*(dx) + pd(3)*(th) + pd(4)*(dth);
u(abs(u)>umax) = sign(u)*umax;
u(abs(th)>1.57) = 0; % u = 0 when abs(th) > 90 degrees

% Linear and angular acceleration equations
ddx = (1/((I+m*l^2)*(M+m)-(m*l*cos(th-pi))^2))*((I+m*l^2)*(u-b*dx) + ...
    m*l*(m*l*g*sin(th-pi)*cos(th-pi)+sin(th-pi)*dth^2));
ddth = (-1/(I+m*l^2))*(m*l*ddx*cos(th-pi) + m*g*l*sin(th-pi));

dydt = zeros(4,1);
dydt = [dx; ddx; dth; ddth];