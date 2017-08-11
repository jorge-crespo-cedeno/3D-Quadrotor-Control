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

persistent waypoints0 traj_time d0
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
        desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%


%% Fill in your code here
persistent cnt
if nargin > 2
    cnt = 0;
end
%         if (t >= (traj_time(2)-0.001)) or (cnt > 0)
%             disp(cnt);
%             disp(t);
%             cnt = cnt + 1;
%         end

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;

persistent alpha_c T S0 S yaw_c

if nargin > 2
    % disp(waypoints);
    waypoints0 = waypoints;
end

[m,n] = size(waypoints0);
% disp(n)
n = n-1;

if nargin > 2
    alpha_c = zeros(8*n, 3);
    for j = 1:3
        A = zeros(8*n, 8*n);

        T = zeros(n, 1) + 3;
        % T = traj_time(1,2:length(traj_time))';
        for i = 1:n
            T(i) = traj_time(i+1) - traj_time(i);
        end
        S0 = 0;
        S = zeros(n, 1);
        for i = 1:n
            S(i) = sum(T(1:i));
        end

        V = zeros(n, 1);
        V(1) = S(1) - S0;
        for i = 2:n
            V(i) = S(i) - S(i-1);
        end

        b = zeros(8*n, 1);

        for i = 1:n
            % pi(Si-1) = wi-1
            A(2*i - 1, (i-1)*8 + 1) = 1;
            b(2*i - 1) = waypoints0(j, i);

            % pi(Si) = wi
            A(2*i, (i-1)*8 + 1) = 1;
            A(2*i, (i-1)*8 + 2) = V(i)/T(i);
            A(2*i, (i-1)*8 + 3) = (V(i)/T(i))^2;
            A(2*i, (i-1)*8 + 4) = (V(i)/T(i))^3;
            A(2*i, (i-1)*8 + 5) = (V(i)/T(i))^4;
            A(2*i, (i-1)*8 + 6) = (V(i)/T(i))^5;
            A(2*i, (i-1)*8 + 7) = (V(i)/T(i))^6;
            A(2*i, (i-1)*8 + 8) = (V(i)/T(i))^7;
            b(2*i) = waypoints0(j, i+1);
        end

        % p1_d(S0) = 0
        A(2*n+1, 2) = 1/T(1);

        % p1_dd(S0) = 0
        A(2*n+2, 3) = 2/(T(1)^2);

        % p1_ddd(S0) = 0
        A(2*n+3, 4) = factorial(3)/(T(1)^3);

        % pn_d(Sn) = 0
        A(2*n+4, 8*(n-1) + 1) = 0;
        A(2*n+4, 8*(n-1) + 2) = 1/T(n);
        A(2*n+4, 8*(n-1) + 3) = (2/T(n))*(V(n)/T(n));
        A(2*n+4, 8*(n-1) + 4) = (3/T(n))*((V(n)/T(n))^2);
        A(2*n+4, 8*(n-1) + 5) = (4/T(n))*((V(n)/T(n))^3);
        A(2*n+4, 8*(n-1) + 6) = (5/T(n))*((V(n)/T(n))^4);
        A(2*n+4, 8*(n-1) + 7) = (6/T(n))*((V(n)/T(n))^5);
        A(2*n+4, 8*(n-1) + 8) = (7/T(n))*((V(n)/T(n))^6);

        % pn_dd(Sn) = 0
        A(2*n+5, 8*(n-1) + 3) = 2/(T(n)^2);
        A(2*n+5, 8*(n-1) + 4) = (factorial(3)/(T(n)^2))*(V(n)/T(n));
        A(2*n+5, 8*(n-1) + 5) = (12/(T(n)^2))*((V(n)/T(n))^2);
        A(2*n+5, 8*(n-1) + 6) = (20/(T(n)^2))*((V(n)/T(n))^3);
        A(2*n+5, 8*(n-1) + 7) = (30/(T(n)^2))*((V(n)/T(n))^4);
        A(2*n+5, 8*(n-1) + 8) = (42/(T(n)^2))*((V(n)/T(n))^5);

        % pn_ddd(Sn) = 0
        A(2*n+6, 8*(n-1) + 4) = factorial(3)/(T(n)^3);
        A(2*n+6, 8*(n-1) + 5) = (factorial(4)/(T(n)^3))*(V(n)/T(n));
        A(2*n+6, 8*(n-1) + 6) = (60/(T(n)^3))*((V(n)/T(n))^2);
        A(2*n+6, 8*(n-1) + 7) = (120/(T(n)^3))*((V(n)/T(n))^3);
        A(2*n+6, 8*(n-1) + 8) = (210/(T(n)^3))*((V(n)/T(n))^4);

        for i = 1:(n-1)
            % pi_d(Si) = pi+1_d(Si)
            A(2*n + 6*i + 1, 8*(i-1) + 2) = 1/T(i);
            A(2*n + 6*i + 1, 8*(i-1) + 3) = (2/T(i))*(V(i)/T(i));
            A(2*n + 6*i + 1, 8*(i-1) + 4) = (3/T(i))*((V(i)/T(i))^2);
            A(2*n + 6*i + 1, 8*(i-1) + 5) = (4/T(i))*((V(i)/T(i))^3);
            A(2*n + 6*i + 1, 8*(i-1) + 6) = (5/T(i))*((V(i)/T(i))^4);
            A(2*n + 6*i + 1, 8*(i-1) + 7) = (6/T(i))*((V(i)/T(i))^5);
            A(2*n + 6*i + 1, 8*(i-1) + 8) = (7/T(i))*((V(i)/T(i))^6);
            A(2*n + 6*i + 1, 8*i + 2) = -1/T(i+1);

            % pi_dd(Si) = pi+1_dd(Si)
            A(2*n + 6*i + 2, 8*(i-1) + 3) = 2/(T(i)^2);
            A(2*n + 6*i + 2, 8*(i-1) + 4) = (factorial(3)/(T(i)^2))*(V(i)/T(i));
            A(2*n + 6*i + 2, 8*(i-1) + 5) = (12/(T(i)^2))*((V(i)/T(i))^2);
            A(2*n + 6*i + 2, 8*(i-1) + 6) = (20/(T(i)^2))*((V(i)/T(i))^3);
            A(2*n + 6*i + 2, 8*(i-1) + 7) = (30/(T(i)^2))*((V(i)/T(i))^4);
            A(2*n + 6*i + 2, 8*(i-1) + 8) = (42/(T(i)^2))*((V(i)/T(i))^5);
            A(2*n + 6*i + 2, 8*i + 3) = -2/(T(i+1)^2);

            % pi_ddd(Si) = pi+1_ddd(Si)
            A(2*n + 6*i + 3, 8*(i-1) + 4) = factorial(3)/(T(i)^3);
            A(2*n + 6*i + 3, 8*(i-1) + 5) = (factorial(4)/(T(i)^3))*(V(i)/T(i));
            A(2*n + 6*i + 3, 8*(i-1) + 6) = (60/(T(i)^3))*((V(i)/T(i))^2);
            A(2*n + 6*i + 3, 8*(i-1) + 7) = (120/(T(i)^3))*((V(i)/T(i))^3);
            A(2*n + 6*i + 3, 8*(i-1) + 8) = (210/(T(i)^3))*((V(i)/T(i))^4);
            A(2*n + 6*i + 3, 8*i + 4) = -factorial(3)/(T(i+1)^3);

            % pi_(4)(Si) = pi+1_(4)(Si)
            A(2*n + 6*i + 4, 8*(i-1) + 5) = factorial(4)/(T(i)^4);
            A(2*n + 6*i + 4, 8*(i-1) + 6) = (factorial(5)/(T(i)^4))*(V(i)/T(i));
            A(2*n + 6*i + 4, 8*(i-1) + 7) = (360/(T(i)^4))*((V(i)/T(i))^2);
            A(2*n + 6*i + 4, 8*(i-1) + 8) = (840/(T(i)^4))*((V(i)/T(i))^3);
            A(2*n + 6*i + 4, 8*i + 5) = -factorial(4)/(T(i+1)^4);

            % pi_(5)(Si) = pi+1_(5)(Si)
            A(2*n + 6*i + 5, 8*(i-1) + 6) = factorial(5)/(T(i)^5);
            A(2*n + 6*i + 5, 8*(i-1) + 7) = (factorial(6)/(T(i)^5))*(V(i)/T(i));
            A(2*n + 6*i + 5, 8*(i-1) + 8) = (2520/(T(i)^5))*((V(i)/T(i))^2);
            A(2*n + 6*i + 5, 8*i + 6) = -factorial(5)/(T(i+1)^5);

            % pi_(6)(Si) = pi+1_(6)(Si)
            A(2*n + 6*i + 6, 8*(i-1) + 7) = factorial(6)/(T(i)^6);
            A(2*n + 6*i + 6, 8*(i-1) + 8) = (factorial(7)/(T(i)^6))*(V(i)/T(i));
            A(2*n + 6*i + 6, 8*i + 7) = -factorial(6)/(T(i+1)^6);
        end

        % alpha_c(:, j) = inv(A)*b;
        alpha_c(:, j) = A\b;
        % alpha is [a10 a11 a12 ... a17 a20 a21 a22 ... an6 an7]'
        % disp('------------b-------------');
        % disp(b);
        % disp('----------alpha-----------');
        % disp(alpha_c);
    end

    %% to calculare yaw_des
%     yaw_waypoints = [0;
%                      0;
%                      0;
%                      0;
%                      0]';
%     yaw_c = zeros(8*n, 1);
% 
%     A = zeros(8*n, 8*n);
% 
%     b = zeros(8*n, 1);
% 
%     for i = 1:n
%         % pi(Si-1) = wi-1
%         A(2*i - 1, (i-1)*8 + 1) = 1;
%         b(2*i - 1) = yaw_waypoints(i);
% 
%         % pi(Si) = wi
%         A(2*i, (i-1)*8 + 1) = 1;
%         A(2*i, (i-1)*8 + 2) = V(i)/T(i);
%         A(2*i, (i-1)*8 + 3) = (V(i)/T(i))^2;
%         A(2*i, (i-1)*8 + 4) = (V(i)/T(i))^3;
%         A(2*i, (i-1)*8 + 5) = (V(i)/T(i))^4;
%         A(2*i, (i-1)*8 + 6) = (V(i)/T(i))^5;
%         A(2*i, (i-1)*8 + 7) = (V(i)/T(i))^6;
%         A(2*i, (i-1)*8 + 8) = (V(i)/T(i))^7;
%         b(2*i) = yaw_waypoints(i+1);
%     end
% 
%     % p1_d(S0) = 0
%     A(2*n+1, 2) = 1/T(1);
% 
%     % p1_dd(S0) = 0
%     A(2*n+2, 3) = 2/(T(1)^2);
% 
%     % p1_ddd(S0) = 0
%     A(2*n+3, 4) = factorial(3)/(T(1)^3);
% 
%     % pn_d(Sn) = 0
%     A(2*n+4, 8*(n-1) + 1) = 0;
%     A(2*n+4, 8*(n-1) + 2) = 1/T(n);
%     A(2*n+4, 8*(n-1) + 3) = (2/T(n))*(V(n)/T(n));
%     A(2*n+4, 8*(n-1) + 4) = (3/T(n))*((V(n)/T(n))^2);
%     A(2*n+4, 8*(n-1) + 5) = (4/T(n))*((V(n)/T(n))^3);
%     A(2*n+4, 8*(n-1) + 6) = (5/T(n))*((V(n)/T(n))^4);
%     A(2*n+4, 8*(n-1) + 7) = (6/T(n))*((V(n)/T(n))^5);
%     A(2*n+4, 8*(n-1) + 8) = (7/T(n))*((V(n)/T(n))^6);
% 
%     % pn_dd(Sn) = 0
%     A(2*n+5, 8*(n-1) + 3) = 2/(T(n)^2);
%     A(2*n+5, 8*(n-1) + 4) = (factorial(3)/(T(n)^2))*(V(n)/T(n));
%     A(2*n+5, 8*(n-1) + 5) = (12/(T(n)^2))*((V(n)/T(n))^2);
%     A(2*n+5, 8*(n-1) + 6) = (20/(T(n)^2))*((V(n)/T(n))^3);
%     A(2*n+5, 8*(n-1) + 7) = (30/(T(n)^2))*((V(n)/T(n))^4);
%     A(2*n+5, 8*(n-1) + 8) = (42/(T(n)^2))*((V(n)/T(n))^5);
% 
%     % pn_ddd(Sn) = 0
%     A(2*n+6, 8*(n-1) + 4) = factorial(3)/(T(n)^3);
%     A(2*n+6, 8*(n-1) + 5) = (factorial(4)/(T(n)^3))*(V(n)/T(n));
%     A(2*n+6, 8*(n-1) + 6) = (60/(T(n)^3))*((V(n)/T(n))^2);
%     A(2*n+6, 8*(n-1) + 7) = (120/(T(n)^3))*((V(n)/T(n))^3);
%     A(2*n+6, 8*(n-1) + 8) = (210/(T(n)^3))*((V(n)/T(n))^4);
% 
%     for i = 1:(n-1)
%         % pi_d(Si) = pi+1_d(Si)
%         A(2*n + 6*i + 1, 8*(i-1) + 2) = 1/T(i);
%         A(2*n + 6*i + 1, 8*(i-1) + 3) = (2/T(i))*(V(i)/T(i));
%         A(2*n + 6*i + 1, 8*(i-1) + 4) = (3/T(i))*((V(i)/T(i))^2);
%         A(2*n + 6*i + 1, 8*(i-1) + 5) = (4/T(i))*((V(i)/T(i))^3);
%         A(2*n + 6*i + 1, 8*(i-1) + 6) = (5/T(i))*((V(i)/T(i))^4);
%         A(2*n + 6*i + 1, 8*(i-1) + 7) = (6/T(i))*((V(i)/T(i))^5);
%         A(2*n + 6*i + 1, 8*(i-1) + 8) = (7/T(i))*((V(i)/T(i))^6);
%         A(2*n + 6*i + 1, 8*i + 2) = -1/T(i+1);
% 
%         % pi_dd(Si) = pi+1_dd(Si)
%         A(2*n + 6*i + 2, 8*(i-1) + 3) = 2/(T(i)^2);
%         A(2*n + 6*i + 2, 8*(i-1) + 4) = (factorial(3)/(T(i)^2))*(V(i)/T(i));
%         A(2*n + 6*i + 2, 8*(i-1) + 5) = (12/(T(i)^2))*((V(i)/T(i))^2);
%         A(2*n + 6*i + 2, 8*(i-1) + 6) = (20/(T(i)^2))*((V(i)/T(i))^3);
%         A(2*n + 6*i + 2, 8*(i-1) + 7) = (30/(T(i)^2))*((V(i)/T(i))^4);
%         A(2*n + 6*i + 2, 8*(i-1) + 8) = (42/(T(i)^2))*((V(i)/T(i))^5);
%         A(2*n + 6*i + 2, 8*i + 3) = -2/(T(i+1)^2);
% 
%         % pi_ddd(Si) = pi+1_ddd(Si)
%         A(2*n + 6*i + 3, 8*(i-1) + 4) = factorial(3)/(T(i)^3);
%         A(2*n + 6*i + 3, 8*(i-1) + 5) = (factorial(4)/(T(i)^3))*(V(i)/T(i));
%         A(2*n + 6*i + 3, 8*(i-1) + 6) = (60/(T(i)^3))*((V(i)/T(i))^2);
%         A(2*n + 6*i + 3, 8*(i-1) + 7) = (120/(T(i)^3))*((V(i)/T(i))^3);
%         A(2*n + 6*i + 3, 8*(i-1) + 8) = (210/(T(i)^3))*((V(i)/T(i))^4);
%         A(2*n + 6*i + 3, 8*i + 4) = -factorial(3)/(T(i+1)^3);
% 
%         % pi_(4)(Si) = pi+1_(4)(Si)
%         A(2*n + 6*i + 4, 8*(i-1) + 5) = factorial(4)/(T(i)^4);
%         A(2*n + 6*i + 4, 8*(i-1) + 6) = (factorial(5)/(T(i)^4))*(V(i)/T(i));
%         A(2*n + 6*i + 4, 8*(i-1) + 7) = (360/(T(i)^4))*((V(i)/T(i))^2);
%         A(2*n + 6*i + 4, 8*(i-1) + 8) = (840/(T(i)^4))*((V(i)/T(i))^3);
%         A(2*n + 6*i + 4, 8*i + 5) = -factorial(4)/(T(i+1)^4);
% 
%         % pi_(5)(Si) = pi+1_(5)(Si)
%         A(2*n + 6*i + 5, 8*(i-1) + 6) = factorial(5)/(T(i)^5);
%         A(2*n + 6*i + 5, 8*(i-1) + 7) = (factorial(6)/(T(i)^5))*(V(i)/T(i));
%         A(2*n + 6*i + 5, 8*(i-1) + 8) = (2520/(T(i)^5))*((V(i)/T(i))^2);
%         A(2*n + 6*i + 5, 8*i + 6) = -factorial(5)/(T(i+1)^5);
% 
%         % pi_(6)(Si) = pi+1_(6)(Si)
%         A(2*n + 6*i + 6, 8*(i-1) + 7) = factorial(6)/(T(i)^6);
%         A(2*n + 6*i + 6, 8*(i-1) + 8) = (factorial(7)/(T(i)^6))*(V(i)/T(i));
%         A(2*n + 6*i + 6, 8*i + 7) = -factorial(6)/(T(i+1)^6);
%     end
% 
%     % yaw_c = inv(A)*b;
%     yaw_c = A\b;
%     % yaw_c is [a10 a11 a12 ... a17 a20 a21 a22 ... an6 an7]'
%     % disp('----------yaw_c------------');
%     % disp(yaw_c);
end

if not(isempty(t))
    if (S0 <= t) && (t <= S(1))
        i = 1;
        x = t - S0;
    else
        for k = 2:n
            if (S(k-1) < t) && (t <= S(k))
                i = k;
                break;
            end
        end
        x = t - S(i-1);
    end
    x = x / T(i);
    for j = 1:3
        r = alpha_c(8*(i-1) + 1, j) + alpha_c(8*(i-1) + 2, j)*x + alpha_c(8*(i-1) + 3, j)*(x^2) + alpha_c(8*(i-1) + 4, j)*(x^3) + alpha_c(8*(i-1) + 5, j)*(x^4) + alpha_c(8*(i-1) + 6, j)*(x^5) + alpha_c(8*(i-1) + 7, j)*(x^6) + alpha_c(8*(i-1) + 8, j)*(x^7);
        desired_state.pos(j) = r;
        r = (alpha_c(8*(i-1) + 2, j) + 2*alpha_c(8*(i-1) + 3, j)*x + 3*alpha_c(8*(i-1) + 4, j)*(x^2) + 4*alpha_c(8*(i-1) + 5, j)*(x^3) + 5*alpha_c(8*(i-1) + 6, j)*(x^4) + 6*alpha_c(8*(i-1) + 7, j)*(x^5) + 7*alpha_c(8*(i-1) + 8, j)*(x^6))/T(i);
        desired_state.vel(j) = r;
        r = (2*alpha_c(8*(i-1) + 3, j) + factorial(3)*alpha_c(8*(i-1) + 4, j)*x + 12*alpha_c(8*(i-1) + 5, j)*(x^2) + 20*alpha_c(8*(i-1) + 6, j)*(x^3) + 30*alpha_c(8*(i-1) + 7, j)*(x^4) + 42*alpha_c(8*(i-1) + 8, j)*(x^5))/(T(i)^2);
        desired_state.acc(j) = r;
    end
end

% if not(isempty(t))
%     if (S0 <= t) && (t < S(1))
%         i = 1;
%         x = t - S0;
%     else
%         for k = 2:n
%             if (S(k-1) <= t) && (t < S(k))
%                 i = k;
%                 break;
%             end
%         end
%         x = t - S(i-1);
%     end
%     x = x / T(i);
%     r = yaw_c(8*(i-1) + 1) + yaw_c(8*(i-1) + 2)*x + yaw_c(8*(i-1) + 3)*(x^2) + yaw_c(8*(i-1) + 4)*(x^3) + yaw_c(8*(i-1) + 5)*(x^4) + yaw_c(8*(i-1) + 6)*(x^5) + yaw_c(8*(i-1) + 7)*(x^6) + yaw_c(8*(i-1) + 8)*(x^7);
%     desired_state.yaw = r;
%     r = (yaw_c(8*(i-1) + 2) + 2*yaw_c(8*(i-1) + 3)*x + 3*yaw_c(8*(i-1) + 4)*(x^2) + 4*yaw_c(8*(i-1) + 5)*(x^3) + 5*yaw_c(8*(i-1) + 6)*(x^4) + 6*yaw_c(8*(i-1) + 7)*(x^5) + 7*yaw_c(8*(i-1) + 8)*(x^6))/T(i);
%     desired_state.yawdot = r;
% end
