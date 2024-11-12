function [F, M, trpy, drpy] = pid_controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

persistent gd;
persistent icnt;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;
 
% =================== Your code starts here ===================
%% Parameter Initialization
params;
params.Kp_gain_euler = [0.5;0.5;0.5];
params.Kd_gain_euler = [0.1;0.1;0.1];

params.Kp_gain_pos = [0.80; 0.82; 0.16];
params.Kd_gain_pos = [0.1;0.1;0.3];

% params.PD_gain_x = [1,0.5];
% params.PD_gain_y = [1, 0.5];
% params.PD_gain_z = [1, 0.5];

qd{qn}.pos;
qd{qn}.vel;
qd{qn}.euler;
qd{qn}.omega;

qd{qn}.pos_des;
qd{qn}.vel_des;
qd{qn}.acc_des;
qd{qn}.yaw_des;
qd{qn}.yawdot_des;

r_ddt_des = qd{qn}.acc_des + params.Kp_gain_pos .* ( qd{qn}.pos_des - qd{qn}.pos ) + params.Kd_gain_pos .* (qd{qn}.vel_des - qd{qn}.vel);
u1 = params.mass * params.grav + params.mass * r_ddt_des(3);
% ===================== Calculate desired YPR==================%
roll_des = 1 / params.grav * ( r_ddt_des(1) * sin(qd{qn}.yaw_des) - r_ddt_des(2) * cos(qd{qn}.yaw_des) );
pitch_des = 1 / params.grav * ( r_ddt_des(1) * cos(qd{qn}.yaw_des) - r_ddt_des(2) * sin(qd{qn}.yaw_des) );
yaw_des =  qd{qn}.yaw_des;
euler_des = [roll_des; pitch_des; yaw_des];

p_des = 0;
q_des = 0;
r_des = qd{qn}.yawdot_des;
euler_vel_des = [p_des; q_des; r_des];

% roll_corr = PD_gain_roll(1) * (euler_des - qd{qn}.euler) + PD_gain_roll(2) * (euler_vel_des - qd{qn}.vel_des);
% pitch_corr = PD_gain_pitch(1) * (euler_des - qd{qn}.euler) + PD_gain_pitch(2) * (euler_vel_des - qd{qn}.vel_des);

corr_euler = params.Kp_gain_euler .* (euler_des - qd{qn}.euler) + params.Kd_gain_euler .* (euler_vel_des - qd{qn}.omega );
u2 = params.I * corr_euler;    

F = u1;
M = corr_euler;

% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end
