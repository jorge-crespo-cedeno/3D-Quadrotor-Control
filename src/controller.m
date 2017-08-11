function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);

% error in position
ep = des_state.pos - state.pos;

% error in velocity
ev = des_state.vel - state.vel;

% K's
Kpx = 20;
Kdx = 1;
Kpy = 20;
Kdy = 1;
Kpz = 15;
Kdz = 8;

Kpphi = 5;
Kdphi = 0.45;
Kptheta = 5;
Kdtheta = 0.45;
Kppsi = 5;
Kdpsi = 0.45;

Kp_angles = [Kpphi Kptheta Kppsi];
Kd_angles = [Kdphi Kdtheta Kdpsi];

% commanded accedelations
x_c_ddot = des_state.acc(1) + Kpx*ep(1) + Kdx*ev(1);
y_c_ddot = des_state.acc(2) + Kpy*ep(2) + Kdy*ev(2);
z_c_ddot = des_state.acc(3) + Kpz*ep(3) + Kdz*ev(3);

% u1
u1 = params.mass*(params.gravity + z_c_ddot);

% angles_c
phi_c = (x_c_ddot*sin(des_state.yaw) - y_c_ddot*cos(des_state.yaw))/params.gravity;
theta_c = (x_c_ddot*cos(des_state.yaw) + y_c_ddot*sin(des_state.yaw))/params.gravity;
angles_c = [phi_c; theta_c; des_state.yaw];

% x_c_dddot
x_c_dddot = Kpx*ev(1) + Kdx*(des_state.acc(1) - x_c_ddot);

% y_c_dddot
y_c_dddot = Kpy*ev(2) + Kdy*(des_state.acc(2) - y_c_ddot);

% angles_dot_c
phi_dot_c = ((x_c_ddot*cos(des_state.yaw) + y_c_ddot*sin(des_state.yaw))*des_state.yawdot + x_c_dddot*sin(des_state.yaw) - y_c_dddot*cos(des_state.yaw))/params.gravity;
theta_dot_c = ((y_c_ddot*cos(des_state.yaw) - x_c_ddot*sin(des_state.yaw))*des_state.yawdot + x_c_dddot*cos(des_state.yaw) + y_c_dddot*sin(des_state.yaw))/params.gravity;
angles_dot_c = [phi_dot_c; theta_dot_c; des_state.yawdot];

% from angle_dot to angular velocities
R_a2w=[cos(theta_c) 0 -cos(phi_c)*sin(theta_c); 0 1 sin(phi_c); sin(theta_c) 0 cos(phi_c)*cos(theta_c)];
w_c = R_a2w*angles_dot_c;

u2 = diag(Kp_angles)*(angles_c - state.rot) + diag(Kd_angles)*(w_c - state.omega);

% disp(u1);
% disp(u2);
F = u1;
M = u2;
% =================== Your code ends here ===================

end
