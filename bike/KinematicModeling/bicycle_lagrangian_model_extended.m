% MATLAB脚本：bicycle_lagrangian_model_extended.m

% 清空工作区和命令窗口
clear;
clc;

% 定义系统的广义坐标
syms theta(t) phi(t) % theta: 偏航角，phi: 前轮转动角
syms d_theta d_phi % 阻尼系数

% 定义广义速度
theta_dot = diff(theta, t);
phi_dot = diff(phi, t);

% 定义已知的参数
total_mass = 15.79; % kg，总质量
g = 9.81; % m/s^2，重力加速度

% 定义整体质心位置
overall_com_y = -0.103; % m
overall_com_z = -0.019; % m

% 定义整体系统的惯性矩
Ixx = 0.8161; Iyy = 2.3674; Izz = 3.0907; 
Ixy = 0.3963; Ixz = 0.0416; Iyz = 0.0033;

% 惯性张量矩阵
I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];

% 计算系统的动能
% 转动动能
omega = [0; theta_dot; phi_dot]; % 假设绕y轴和z轴的转动
T_rot = 0.5 * omega' * I * omega;

% 平动动能
v_com = [0; overall_com_y * theta_dot; overall_com_z * phi_dot];
T_trans = 0.5 * total_mass * (v_com' * v_com);

% 总动能
T = T_rot + T_trans;

% 计算系统的势能
V = total_mass * g * overall_com_z;

% 拉格朗日量
L = T - V;

% 计算拉格朗日方程的导数
dL_dtheta_dot = diff(L, diff(theta, t));
dL_dtheta = diff(L, theta);
dL_dphi_dot = diff(L, diff(phi, t));
dL_dphi = diff(L, phi);

% 对时间求导
d_dt_dL_dtheta_dot = diff(dL_dtheta_dot, t);
d_dt_dL_dphi_dot = diff(dL_dphi_dot, t);

% 路面影响模拟（模拟坡度和表面粗糙度的附加阻尼）
road_slope_angle = pi / 18; % 10度坡度
additional_damping = 0.02; % 模拟粗糙度产生的附加阻尼

% 添加阻尼项
damping_theta = (d_theta + additional_damping) * theta_dot + total_mass * g * sin(road_slope_angle);
damping_phi = (d_phi + additional_damping) * phi_dot;

% 控制力矩（PID控制模拟驾驶员的操控）
Kp_theta = 1.0; Kd_theta = 0.5; % PID参数
Kp_phi = 0.8; Kd_phi = 0.3; % PID参数
control_torque_theta = Kp_theta * theta + Kd_theta * theta_dot;
control_torque_phi = Kp_phi * phi + Kd_phi * phi_dot;

% 构造拉格朗日方程，添加阻尼和控制力矩
eq1 = d_dt_dL_dtheta_dot - dL_dtheta + damping_theta == control_torque_theta;
eq2 = d_dt_dL_dphi_dot - dL_dphi + damping_phi == control_torque_phi;

% 显示拉格朗日方程的结果
disp('拉格朗日方程（包含阻尼、路面影响和控制力矩）：');
eq1 = simplify(eq1)
eq2 = simplify(eq2)

% 设置不同阻尼系数的数值范围
damping_values = [0.05, 0.1, 0.2];

% 解析拉格朗日方程矩阵
A = [253491611/200000000, 33/20000;
     33/20000, 309640019/200000000];

% 初始条件
initial_conditions = [0; 0; 0; 0]; % 初始角度和角速度
time_span = [0 10];

% 遍历不同的阻尼系数
for i = 1:length(damping_values)
    d_theta_val = damping_values(i);
    d_phi_val = damping_values(i);

    % 定义外部力矩向量 Q（包括阻尼和控制力矩）
    Q = [double(subs(control_torque_theta, theta, 0) - d_theta_val * 0);
         double(subs(control_torque_phi, phi, 0) - d_phi_val * 0)];

    % 数值积分函数
    odefun = @(t, y) [y(3); y(4); A \ (Q - [(d_theta_val + additional_damping) * y(3) + total_mass * g * sin(road_slope_angle);
                                             (d_phi_val + additional_damping) * y(4)])];

    % 使用 ode45 进行数值积分
    [t, Y] = ode45(odefun, time_span, initial_conditions);

    % 绘制角度随时间的变化
    figure;
    plot(t, Y(:, 1), 'r-', 'DisplayName', 'Theta (偏航角)');
    hold on;
    plot(t, Y(:, 2), 'b-', 'DisplayName', 'Phi (前轮角)');
    xlabel('时间 (s)');
    ylabel('角度 (rad)');
    legend;
    title(['自行车动态行为模拟（阻尼系数 = ', num2str(damping_values(i)), '）']);
end

% 输出数学公式
fprintf('拉格朗日方程（包含阻尼、路面影响和控制力矩）为：\n');
fprintf('eq1: d/dt(dL/d(theta_dot)) - dL/d(theta) + damping_theta = control_torque_theta\n');
fprintf('eq2: d/dt(dL/d(phi_dot)) - dL/d(phi) + damping_phi = control_torque_phi\n');
fprintf('其中：控制力矩为：\n');
fprintf('control_torque_theta = Kp_theta * theta + Kd_theta * theta_dot\n');
fprintf('control_torque_phi = Kp_phi * phi + Kd_phi * phi_dot\n');
fprintf('路面影响：坡度角 = %.2f 度，附加阻尼 = %.2f\n', rad2deg(road_slope_angle), additional_damping);
