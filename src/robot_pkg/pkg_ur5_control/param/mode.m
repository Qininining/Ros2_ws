% 清除工作区和图形
clear all; close all; clc;
clf;
hold on;
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Robot Arm Visualization');
view(3);

% 关节名称（按从基座到末端的顺序）
joints = {'shoulder', 'upper_arm', 'forearm', 'wrist_1', 'wrist_2', 'wrist_3'};

% 关节标定数据（相对于前一个关节的变换）
data.shoulder = [0, 0, 0.089201138580847966, -0, 0, 1.5413963267648517e-05];
data.upper_arm = [8.28481932000539e-05, 0, 0, 1.5702793305568989, 0, 9.4248239648454113e-05];
data.forearm = [-0.42514603197027351, 0, 0, 0.00027727163297603633, 0.00074181659174685764, 8.2061777589927939e-05];
data.wrist_1 = [-0.39231335412023205, 0.00070367842866065884, 0.11004902881674888, 3.1351985141209435, -3.1400954402443415, 3.1415185610293745];
data.wrist_2 = [3.6501405590628345e-05, -0.094721186124807052, 0.0001791428279527357, 1.5689050644312028, 0, -1.320607040336194e-05];
data.wrist_3 = [0.00015399008043081661, 0.082421327089370319, 0.00012632686207930083, 1.5723290218826846, 3.1415926535897931, -3.14158677108863];

% 初始化变换矩阵（基座到世界坐标系）
T_accum = eye(4);

% 存储所有关节在基坐标系中的位置（仅包括关节）
positions = [];

% 遍历每个关节计算累积变换
for i = 1:numel(joints)
    joint = joints{i};
    d = data.(joint);
    
    % 提取位置和欧拉角
    x = d(1); y = d(2); z = d(3);
    roll = d(4); pitch = d(5); yaw = d(6);
    
    % 计算旋转矩阵（RPY: Z-Y-X顺序）
    Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
    Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
    Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    R = Rz * Ry * Rx;
    
    % 构建齐次变换矩阵
    T_rel = [R, [x; y; z]; [0, 0, 0, 1]];
    
    % 更新累积变换（相对于基座）
    T_accum = T_accum * T_rel;
    % 打印当前关节的变换矩阵
    % fprintf('Transformation Matrix for %s:\n', joint);
    % disp(T_accum);

    % 提取当前关节的旋转矩阵和位置
    R_current = T_accum(1:3, 1:3);
    temp_Z = [0; 0; 1]; % 假设关节旋转轴为Z轴
    % 计算当前关节Z轴在基座坐标系中的表示
    Z_base = R_current * temp_Z;
    % 定义绕Z轴旋转180的R
    R_180 = [-1, 0, 0; 0, -1, 0; 0, 0, 1];
    % 计算Z轴在基座坐标系中的表示
    Z_base_1 = R_180 * Z_base;
    % 打印当前关节Z轴在基座坐标系中的表示
    fprintf('Z-axis in base frame for %s: [%.16f, %.16f, %.16f]\n', joint, Z_base_1(1), Z_base_1(2), Z_base_1(3));

    % Z-axis in base frame for shoulder: [0.000000, 0.000000, 1.000000]
    % Z-axis in base frame for upper_arm: [-0.000110, 1.000000, 0.000517]
    % Z-axis in base frame for forearm: [-0.000852, 1.000000, 0.000240]
    % Z-axis in base frame for wrist_1: [0.000646, 0.999978, 0.006634]
    % Z-axis in base frame for wrist_2: [0.000011, 0.008525, -0.999964]
    % Z-axis in base frame for wrist_3: [0.000652, 0.999975, 0.006992]

    % 提取当前关节在基座中的位置
    current_pos = T_accum(1:3, 4)';
    % 打印当前关节位置
    fprintf('Position of %s in base frame: [%.16f, %.16f, %.16f]\n', joint, current_pos(1), current_pos(2), current_pos(3));

    positions = [positions; current_pos];
end

% 绘制关节位置
plot3(positions(:,1), positions(:,2), positions(:,3), 'o-', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'r');

% 标记关键点
text(positions(1,1), positions(1,2), positions(1,3), 'Joint 1', 'FontSize', 10);
text(positions(2,1), positions(2,2), positions(2,3), 'Joint 2', 'FontSize', 10);
text(positions(3,1), positions(3,2), positions(3,3), 'Joint 3', 'FontSize', 10);
text(positions(4,1), positions(4,2), positions(4,3), 'Joint 4', 'FontSize', 10);
text(positions(5,1), positions(5,2), positions(5,3), 'Joint 5', 'FontSize', 10);
text(positions(6,1), positions(6,2), positions(6,3), 'End Effector', 'FontSize', 10);

% 添加坐标轴
quiver3(0, 0, 0, 0.1, 0, 0, 'r', 'LineWidth', 2); % X轴
quiver3(0, 0, 0, 0, 0.1, 0, 'g', 'LineWidth', 2); % Y轴
quiver3(0, 0, 0, 0, 0, 0.1, 'b', 'LineWidth', 2); % Z轴
legend('Robot Arm', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'End Effector', 'X', 'Y', 'Z');

hold off;



