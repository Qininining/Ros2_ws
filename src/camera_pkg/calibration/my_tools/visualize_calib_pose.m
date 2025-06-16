% 清理工作区和命令行窗口
clear;
clc;
close all;

% --- 1. 定义文件路径 ---
calib_file_name = 'ur5_l515_eih_calib.calib';

% --- 2. 尝试读取文件内容 ---
try
    file_content = fileread(calib_file_name);
catch ME
    if strcmp(ME.identifier, 'MATLAB:fileread:CannotOpenFile')
        fprintf('错误：无法打开文件 "%s"。请确保文件位于当前MATLAB目录中。\n', calib_file_name);
        % 如果文件不存在，则创建虚拟内容进行演示
        fprintf('为演示目的，将使用硬编码的位姿数据进行可视化。\n');
        file_content = sprintf([...
            'parameters:\n'...
            '  name: ur5_l515_eih_calib\n'...
            '  calibration_type: eye_in_hand\n'...
            '  robot_base_frame: base_link\n'...
            '  robot_effector_frame: tool0\n'...
            '  tracking_base_frame: camera_color_optical_frame\n'...
            '  tracking_marker_frame: aruco_marker_0\n'...
            '  freehand_robot_movement: true\n'...
            '  move_group_namespace: /\n'...
            '  move_group: manipulator\n'...
            'transform:\n'...
            '  translation:\n'...
            '    x: -0.10141114329981354\n'...
            '    y: 0.08558407907548507\n'...
            '    z: 0.020387209628634714\n'...
            '  rotation:\n'...
            '    x: 0.06322367934689543\n'...
            '    y: 0.06481344784792709\n'...
            '    z: -0.3738340529057366\n'...
            '    w: 0.9230655904300494\n'...
            ]);
    else
        rethrow(ME); % 重新抛出其他错误
    end
end

% --- 3. 解析文件内容以提取位姿信息 ---
% 初始化变量
translation_vec = [0, 0, 0];
rotation_quat = [0, 0, 0, 1]; % [x, y, z, w] 形式

% 使用正则表达式查找和提取数值
% 提取平移
translation_pattern = 'translation:\s*x:\s*([-\d.]+)\s*y:\s*([-\d.]+)\s*z:\s*([-\d.]+)';
tokens = regexp(file_content, translation_pattern, 'tokens', 'once');
if ~isempty(tokens)
    translation_vec = [str2double(tokens{1}), str2double(tokens{2}), str2double(tokens{3})];
else
    fprintf('警告：未在文件中找到平移数据。\n');
end

% 提取旋转 (四元数)
rotation_pattern = 'rotation:\s*x:\s*([-\d.]+)\s*y:\s*([-\d.]+)\s*z:\s*([-\d.]+)\s*w:\s*([-\d.]+)';
tokens = regexp(file_content, rotation_pattern, 'tokens', 'once');
if ~isempty(tokens)
    rotation_quat = [str2double(tokens{1}), str2double(tokens{2}), str2double(tokens{3}), str2double(tokens{4})];
else
    fprintf('警告：未在文件中找到旋转数据。\n');
end

fprintf('已解析的平移 (x, y, z): [%.6f, %.6f, %.6f]\n', translation_vec);
fprintf('已解析的旋转 (x, y, z, w): [%.6f, %.6f, %.6f, %.6f]\n', rotation_quat);

% --- 4. 将四元数转换为旋转矩阵 ---
% MATLAB 的 quaternion 函数接受 [w, x, y, z] 或 [x, y, z, w] 格式。
% 确保四元数是单位四元数
q = quaternion(rotation_quat(4), rotation_quat(1), rotation_quat(2), rotation_quat(3)); % (w, x, y, z) 顺序
R = rotmat(q, 'point'); % 旋转矩阵

% --- 5. 构建齐次变换矩阵 ---
T = eye(4);
T(1:3, 1:3) = R;
T(1:3, 4) = translation_vec'; % 平移向量作为最后一列

fprintf('\n生成的齐次变换矩阵 T:\n');
disp(T);

% --- 6. 可视化位姿 ---
figure('Name', '位姿可视化');
hold on;
axis equal;
grid on;
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');
title('位姿可视化');

% 设置坐标轴范围以便更好地查看
axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);

% 检查 Robotics System Toolbox 是否可用
if license('checkout', 'Robotics_System_Toolbox') && exist('plotTransforms', 'file') == 2
    % 使用 plotTransforms 函数可视化坐标系
    % Origin 是平移向量
    % R 是旋转矩阵 (MATLAB 的 plotTransforms 期望列向量作为旋转矩阵，所以这里直接使用 R)
    plotTransforms(translation_vec, q, 'FrameSize', 0.1, 'Parent', gca); % 'FrameSize' 控制坐标轴的长度
    text(translation_vec(1), translation_vec(2), translation_vec(3) + 0.05, '目标位姿', 'FontSize', 10, 'Color', 'b');
    
    % 绘制世界坐标系原点
    plotTransforms([0,0,0], quaternion([1,0,0,0]), 'FrameSize', 0.1, 'Parent', gca); % 移除 'Labels' 参数
    text(0,0,0.05, '世界原点', 'FontSize', 10, 'Color', 'k');

    fprintf('\n已使用 Robotics System Toolbox 的 plotTransforms 函数可视化位姿。\n');

else
    fprintf('\n警告：未检测到 Robotics System Toolbox 或 plotTransforms 函数。将使用基本 plot3 进行可视化。\n');
    
    % 如果没有 Robotics System Toolbox，手动绘制坐标系
    origin = translation_vec;
    
    % 绘制原点
    plot3(origin(1), origin(2), origin(3), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    % 绘制坐标轴 (使用旋转矩阵的列向量作为轴方向)
    quiver3(origin(1), origin(2), origin(3), R(1,1), R(2,1), R(3,1), 0.1, 'r', 'LineWidth', 2); % X轴 (红色)
    quiver3(origin(1), origin(2), origin(3), R(1,2), R(2,2), R(3,2), 0.1, 'g', 'LineWidth', 2); % Y轴 (绿色)
    quiver3(origin(1), origin(2), origin(3), R(1,3), R(2,3), R(3,3), 0.1, 'b', 'LineWidth', 2); % Z轴 (蓝色)
    
    text(origin(1), origin(2), origin(3) + 0.05, '目标位姿', 'FontSize', 10, 'Color', 'b');
    
    % 绘制世界坐标系原点
    quiver3(0, 0, 0, 0.1, 0, 0, 0.1, 'r', 'LineWidth', 2, 'ShowArrowHead', 'off'); % X
    quiver3(0, 0, 0, 0, 0.1, 0, 0.1, 'g', 'LineWidth', 2, 'ShowArrowHead', 'off'); % Y
    quiver3(0, 0, 0, 0, 0, 0.1, 0.1, 'b', 'LineWidth', 2, 'ShowArrowHead', 'off'); % Z
    text(0,0,0.05, '世界原点', 'FontSize', 10, 'Color', 'k');
end

view(3); % 设置为 3D 视角
hold off;

fprintf('\n脚本执行完毕。\n');
