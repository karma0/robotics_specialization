function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% FILL IN YOUR CODE HERE
Kp_z = 200;
Kv_z = 20;
Kp_y = 10;
Kv_y = 10;

% This loop takes phi_c and phidot_c and produces u2
Kp_phi = 1000; % Inner loop must be very fast compared to outer loop
Kv_phi = 50;

Ixx = params.Ixx;
g = params.gravity;
m = params.mass;

y = state.pos(1);
z = state.pos(2);
y_dot = state.vel(1);
z_dot = state.vel(2);

d_y = des_state.pos(1);
d_z = des_state.pos(2);
d_y_dot = des_state.vel(1);
d_z_dot = des_state.vel(2);

d_y_ddot = des_state.acc(1);
d_z_ddot = des_state.acc(2);

phi = state.rot(1);
phi_dot = state.omega(1);

Ep_y = d_y - y;
Ep_z = d_z - z;
Ev_y = d_y_dot - y_dot;
Ev_z = d_z_dot - z_dot;

phi_c = -(1/g)*(d_y_ddot + Kv_y*Ev_y + Kp_y*Ep_y);
phi_c_dot = 0;

Ep_phi = phi_c - phi;
Ev_phi_dot = phi_c_dot - phi_dot;
 
u1 = m*(g + d_z_ddot + Kv_z*Ev_z + Kp_z*Ep_z);
u2 = Ixx*(Kv_phi*Ev_phi_dot + Kp_phi*Ep_phi);

end
