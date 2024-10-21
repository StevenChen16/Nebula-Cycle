% MATLAB脚本：bicycle_kinematics_basic.m

% 清空工作区和命令窗口
clear;
clc;

% 单位转换
mm_to_m = 1e-3; % 毫米到米
g_to_kg = 1e-3; % 克到千克

% 定义各部件的质量 (kg)
frame_mass = 4599.152 * g_to_kg;
fork_handlebar_mass = 3274.571 * g_to_kg;
front_tire_mass = 2150.007 * g_to_kg;
back_tire_mass = 2964.786 * g_to_kg;
drivetrain_mass = 1941.898 * g_to_kg;
seat_mass = 857.211 * g_to_kg;

% 定义各部件的质心位置 (m)
frame_com = [65.233, -54.373, -17.346] * mm_to_m;
fork_handlebar_com = [-394.198, 76.295, -14.326] * mm_to_m;
front_tire_com = [-433.018, -257.574, -1.419] * mm_to_m;
back_tire_com = [566.246, -277.365, -25.046] * mm_to_m;
drivetrain_com = [42.082, -226.459, -40.891] * mm_to_m;
seat_com = [196.846, 213.817, -26.019] * mm_to_m;

% 计算系统的总质量
total_mass = frame_mass + fork_handlebar_mass + front_tire_mass + ...
    back_tire_mass + drivetrain_mass + seat_mass;

% 计算系统的整体质心位置 (m)
overall_com = (frame_mass * frame_com + ...
    fork_handlebar_mass * fork_handlebar_com + ...
    front_tire_mass * front_tire_com + ...
    back_tire_mass * back_tire_com + ...
    drivetrain_mass * drivetrain_com + ...
    seat_mass * seat_com) / total_mass;

% 显示总质量和整体质心位置
fprintf('总质量: %.2f kg\n', total_mass);
fprintf('整体质心位置: [%.3f, %.3f, %.3f] m\n', overall_com(1), overall_com(2), overall_com(3));

% 定义各部件的惯性矩 (kg*m^2)，质心处
frame_inertia = [
    9.318e+07, 1.135e+08, 2.679e+06;
    1.135e+08, 2.599e+08, -3.258e+05;
    2.679e+06, -3.258e+05, 3.412e+08
] * g_to_kg * (mm_to_m)^2;

fork_handlebar_inertia = [
    1.754e+08, -1.878e+07, 9.599e+05;
    -1.878e+07, 2.531e+07, 7.417e+06;
    9.599e+05, 7.417e+06, 1.608e+08
] * g_to_kg * (mm_to_m)^2;

front_tire_inertia = [
    8.277e+07, -2.029e+07, 2.674e+06;
    -2.029e+07, 8.010e+07, 6.523e+06;
    2.674e+06, 6.523e+06, 1.598e+08
] * g_to_kg * (mm_to_m)^2;

back_tire_inertia = [
    6.477e+07, 3561.186, 2.535e+05;
    3561.186, 6.455e+07, 1.347e+06;
    2.535e+05, 1.347e+06, 1.206e+08
] * g_to_kg * (mm_to_m)^2;

drivetrain_inertia = [
    2.324e+07, 3129.507, -13568.003;
    3129.507, 1.413e+07, -7.897e+06;
    -13568.003, -7.897e+06, 1.323e+07
] * g_to_kg * (mm_to_m)^2;

seat_inertia = [
    1.595e+06, -4.783e+05, 54698.869;
    -4.783e+05, 2.730e+06, 20507.90;
    54698.869, 20507.90, 2.936e+06
] * g_to_kg * (mm_to_m)^2;

% 转换各个部件的惯性矩到整体质心
frame_inertia_total = parallel_axis_theorem(frame_inertia, frame_mass, frame_com, overall_com);
fork_handlebar_inertia_total = parallel_axis_theorem(fork_handlebar_inertia, fork_handlebar_mass, fork_handlebar_com, overall_com);
front_tire_inertia_total = parallel_axis_theorem(front_tire_inertia, front_tire_mass, front_tire_com, overall_com);
back_tire_inertia_total = parallel_axis_theorem(back_tire_inertia, back_tire_mass, back_tire_com, overall_com);
drivetrain_inertia_total = parallel_axis_theorem(drivetrain_inertia, drivetrain_mass, drivetrain_com, overall_com);
seat_inertia_total = parallel_axis_theorem(seat_inertia, seat_mass, seat_com, overall_com);

% 求和计算总的惯性矩
overall_inertia = frame_inertia_total + fork_handlebar_inertia_total + ...
    front_tire_inertia_total + back_tire_inertia_total + ...
    drivetrain_inertia_total + seat_inertia_total;

% 显示整体系统的惯性矩
disp('整体系统的惯性矩（kg*m^2）:');
disp(overall_inertia);

% 辅助函数定义：平行轴定理
function I_total = parallel_axis_theorem(I_local, mass, local_com, overall_com)
    % 计算质心到整体质心的偏移向量
    d = local_com - overall_com;
    % 计算质心偏移的平方和
    d_squared = norm(d)^2;
    % 计算偏移向量的外积矩阵
    d_outer = d' * d;
    % 根据平行轴定理计算转换后的惯性矩
    I_total = I_local + mass * (d_squared * eye(3) - d_outer);
end
