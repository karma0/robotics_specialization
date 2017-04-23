function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
%


%% Fill in your code here

persistent waypoints0 traj_time d0 pos_Cs vel_Cs acc_Cs

if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    traj_time(end)
    waypoints0 = waypoints;

    n = size(waypoints0,2)-1;
    A = zeros(8*n, 8*n);
    b = zeros(8*n, 3);

    % "Left" end of waypoint segment (t=0)
    %    p(t=0) = w_i
    %   -> a_i0 = w_i
    rowctr = 1;
    for i = 0:n-1
        A(rowctr,i*8+8) = 1;
        b(rowctr,:) = waypoints0(:,i+1)';
        rowctr = rowctr + 1;
    end

    C0 = ones(1,8);
    C1 = polyder(C0);
    C2 = polyder(C1);
    C3 = polyder(C2);
    C4 = polyder(C3);
    C5 = polyder(C4);
    C6 = polyder(C5);

    % List of polynomial coefficients up to 6th derivative
    dp_Cs = {C1, C2, C3, C4, C5, C6};

    % "Right" end of waypoint segment (t=1)
    %    p(t=1) = w_(i+1)
    for i = 0:n-1
        A(rowctr,i*8+1:i*8+8) = C0;
        b(rowctr,:) = waypoints0(:,i+2)';
        rowctr = rowctr + 1;
    end

    % pdot(S0) = pdot(Sn) = 0 -- boundary conditions
    A(rowctr,7) = C1(7); rowctr = rowctr + 1;
    A(rowctr,8*(n-1)+1:8*n) = [C1 0]; rowctr = rowctr + 1;
    % Acceleration = 0 at initial and terminal
    A(rowctr,6) = C1(6); rowctr = rowctr + 1;
    A(rowctr,8*(n-1)+1:8*n) = [C2 0 0]; rowctr = rowctr + 1;
    % Jerk = 0  at initial and terminal
    A(rowctr,5) = C1(5); rowctr = rowctr + 1;
    A(rowctr,8*(n-1)+1:8*n) = [C3 0 0 0]; rowctr = rowctr + 1;

    % Continuity of 1st-6th derivative
    % pdot_i(t=1) = pdot_(i+1) (t=0)
    for i = 0:n-2
        for k = 1:6
            A(rowctr,i*8+1:i*8+8) = [dp_Cs{k} zeros(1,k)];
            A(rowctr,i*8+8 + (8-k)) = -dp_Cs{k}(8-k);
            rowctr = rowctr + 1;
        end
    end

    % Compute polynomial coefficients
    all_Cs = A\b;

    pos_Cs = cell(n,1);
    vel_Cs = cell(n,1);
    acc_Cs = cell(n,1);

    % Split coefficients into cell array
    % Also store velocity and acceleration coefficients
    for i=1:n
        cx = all_Cs(((i-1)*8+1):i*8,1);
        cy = all_Cs(((i-1)*8+1):i*8,2);
        cz = all_Cs(((i-1)*8+1):i*8,3);
        pos_Cs{i} = [cx, cy, cz];

        cx1 = polyder(cx)';
        cy1 = polyder(cy)';
        cz1 = polyder(cz)';
        vel_Cs{i} = [cx1, cy1, cz1];

        cx2 = polyder(cx1)';
        cy2 = polyder(cy1)';
        cz2 = polyder(cz1)';
        acc_Cs{i} = [cx2, cy2, cz2];
    end

else

    if(t > traj_time(end))
        t = traj_time(end)-0.001;
    end

    t_index = find(traj_time >= t,1)-1;
    if(t_index > 1)
        t = t - traj_time(t_index);
    end

    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    else
        % Scaled time between 0 and 1
        scale = t/d0(t_index);

        % Get polynomial coefficients for current segment for x, y and z
        seg_Cs = pos_Cs{t_index};
        cx = seg_Cs(:,1);
        cy = seg_Cs(:,2);
        cz = seg_Cs(:,3);

        desired_state.pos = [polyval(cx, scale);
                             polyval(cy, scale);
                             polyval(cz, scale);];
        % Compute value for polynomial for velocity
        seg_Cs = vel_Cs{t_index};
        cx1 = seg_Cs(:,1);
        cy1 = seg_Cs(:,2);
        cz1 = seg_Cs(:,3);
        vel = [polyval(cx1, scale);
               polyval(cy1, scale);
               polyval(cz1, scale);];
        % because of chain rule, we have to divide by T
        desired_state.vel = vel.*(1/d0(t_index));

        % Compute value for polynomial for acceleration
        seg_Cs = acc_Cs{t_index};
        cx2 = seg_Cs(:,1);
        cy2 = seg_Cs(:,2);
        cz2 = seg_Cs(:,3);

        acc = [polyval(cx2, scale);
               polyval(cy2, scale);
               polyval(cz2, scale);];
        % because of chain rule, we have to divide by T
        desired_state.acc = acc.*(1/d0(t_index)^2);
    end
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end


% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

