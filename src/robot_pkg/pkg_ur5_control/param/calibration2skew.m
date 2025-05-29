% 计算 UR5 机械臂各关节的空间旋量
% 该脚本计算 UR5 机械臂各关节的空间旋量，并可视化机械臂的关节位置和旋转轴
% Space Twist for Shoulder:
%      0     0     1     0     0     0

% Space Twist for Upper Arm:
%       -0.00010966218804077         0.999999860344549      0.000516996214966776        -0.089201126124083     -9.73915983020756e-06     -8.28481817600509e-05

% Space Twist for Forearm:
%      -0.000851501411008793         0.999999608724093      0.000239785336181799       -0.0892010924995179     -0.000177878813172699         0.425063054601736

% Space Twist for Wrist 1:
%       0.000645748395716926         0.999977787178547       0.00663386452268502       -0.0891645904058137      -0.00536369075116089         0.817193278688037

% Space Twist for Wrist 2:
%       1.07663873727645e-05        0.0085250671036427        -0.999963660897216        -0.111191909169889         0.817216260164118       0.00696587945472968

% Space Twist for Wrist 3:
%       0.000651627424651037         0.999975340393563       0.00699242350521426         0.005596450592457      -0.00571659008736621         0.816998900818537

% Initial End Effector Pose (M_initial):
%         -0.999999787643124     -9.76765308370577e-06      0.000651627424651037         0.817145243199844
%        0.00065167979368323      -0.00699241862472029         0.999975340393563         0.193656522784246
%      -5.21096047671649e-06         0.999975552694349       0.00699242350521426      -0.00424242678717729
%                          0                         0                         0                         1




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
rotations = [];

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

    % 提取当前关节的旋转矩阵和位置
    R_current = T_accum(1:3, 1:3);
    current_pos = ([-1, 0, 0; 0, -1, 0; 0, 0, 1] * T_accum(1:3, 4))';

    % %打印R_current
    % fprintf('Rotation Matrix for %s:\n', joint);
    % disp(R_current);

    % 存储位置和旋转轴
    positions = [positions; current_pos];
    rotations = [rotations; ([-1, 0, 0; 0, -1, 0; 0, 0, 1] * R_current * [0; 0; 1])']; % 假设关节旋转轴为Z轴

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

% 定义各关节的旋转轴 (单位向量)
z_shoulder = rotations(1,:);
z_upper_arm = rotations(2,:);
z_forearm = rotations(3,:);
z_wrist_1 = rotations(4,:);
z_wrist_2 = rotations(5,:);
z_wrist_3 = rotations(6,:);

% 定义各关节的位置
p_shoulder = positions(1,:);
p_upper_arm = positions(2,:);
p_forearm = positions(3,:);
p_wrist_1 = positions(4,:);
p_wrist_2 = positions(5,:);
p_wrist_3 = positions(6,:);

% 打印各关节的位置和旋转轴
% fprintf('Shoulder Position: [%.16f, %.16f, %.16f], Axis: [%.16f, %.16f, %.16f]\n', p_shoulder, z_shoulder);
% fprintf('Upper Arm Position: [%.16f, %.16f, %.16f], Axis: [%.16f, %.16f, %.16f]\n', p_upper_arm, z_upper_arm);

% 计算各关节的空间旋量
S_shoulder = calculate_spatial_twist(p_shoulder, z_shoulder);
S_upper_arm = calculate_spatial_twist(p_upper_arm, z_upper_arm);
S_forearm = calculate_spatial_twist(p_forearm, z_forearm);
S_wrist_1 = calculate_spatial_twist(p_wrist_1, z_wrist_1);
S_wrist_2 = calculate_spatial_twist(p_wrist_2, z_wrist_2);
S_wrist_3 = calculate_spatial_twist(p_wrist_3, z_wrist_3);

% 显示结果
% 显示小数点后16位
format long g;
disp('Space Twist for Shoulder:');
disp(S_shoulder);
disp('Space Twist for Upper Arm:');
disp(S_upper_arm);
disp('Space Twist for Forearm:');
disp(S_forearm);
disp('Space Twist for Wrist 1:');
disp(S_wrist_1);
disp('Space Twist for Wrist 2:');
disp(S_wrist_2);
disp('Space Twist for Wrist 3:');
disp(S_wrist_3);


% 定义[-1, 0, 0; 0, -1, 0; 0, 0, 1]的齐次变换矩阵
T_Rz_180 = [-1, 0, 0, 0; 
            0, -1, 0, 0; 
            0, 0, 1, 0; 
            0, 0, 0, 1];
% 计算 M_initial标准末端执行器初始位姿 M_initial
M_initial = T_Rz_180 * T_accum;

% 打印 M_initial
disp('Initial End Effector Pose (M_initial):');
disp(M_initial);



function S = calculate_spatial_twist(p, omega_hat)
    % 计算线速度 v_i
    v = cross(p, omega_hat);
    
    % 空间旋量 S_i = [v; omega_hat]
    S = [omega_hat, v];
end

