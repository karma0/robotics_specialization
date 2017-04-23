function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters


Kp = 180;
Kv = 22;

p_error = s_des(1) - s(1);
d_error = s_des(2) - s(2);

u = params.mass * (params.gravity + Kv*d_error + Kp*p_error);

end

