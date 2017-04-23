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

Kp = [30; 30; 80];
Kd = [10; 10; 35];
r_Kp = [200; 200; 200];
r_Kd = [.1; .1; .1];

g = params.gravity;
m = params.mass;

v = state.vel;
p = state.pos;
r = state.rot;
o = state.omega;

vdot = des_state.vel;
pdot = des_state.pos;
adot = des_state.acc;
ydot = des_state.yawdot;

vel_err = vdot - v;
pos_err = pdot - p;

v_alpha = vdot / norm(vdot);
a_alpha = pdot / norm(pdot);

va_alpha = cross(v_alpha, a_alpha);

pose_offset = ((pos_err' * v_alpha) * v_alpha) + ((pos_err' * a_alpha) * a_alpha);

r_ddot_c = adot + (Kd .* vel_err) + (Kp .* pos_err);


% Thrust
F = m*(g + r_ddot_c(3));

% Moment
psi_dot = des_state.yaw;
phi_dot = (r_ddot_c(1) * sin(psi_dot)) - ((r_ddot_c(2) * cos(psi_dot)) / g);
theta_dot = (r_ddot_c(1) * cos(psi_dot)) + ((r_ddot_c(2) * sin(psi_dot)) / g);

des_rot = [ phi_dot; theta_dot; psi_dot ];
des_omega = [ 0; 0; ydot ];

M = (r_Kp .* (des_rot - r)) + (r_Kd .* (des_omega - o));

% =================== Your code ends here ===================

end
