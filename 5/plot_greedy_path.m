function plot_random_path(varargin)
%% 绘制随机生成个体的路径图
% 车辆路径: 黑色实线 (车辆服务节点: 蓝色圆点)
% 无人机A路径: 红色虚线 (服务节点: 红色菱形; 发射/回收点: 红色三角)
% 无人机B路径: 绿色虚线 (服务节点: 绿色方形; 发射/回收点: 绿色三角)

if nargin < 6
    error(['plot_random_path 需要5个输入，收到了 %d 个。\n', ...
        '用法: plot_random_path(coords, vehicle_route, drone_A_tasks, drone_B_tasks, title_str)\n', ...
        '请运行 vehicle_drone_new 主程序来调用此函数。'], nargin);
end

coords         = varargin{1};
vehicle_route  = varargin{2};
drone_A_tasks  = varargin{3};
drone_B_tasks  = varargin{4};
title_str      = varargin{5};
number =varargin{6};

figure('Name', title_str, 'Position', [100, 100, 700, 600]);
hold on;

n_nodes = size(coords, 1);
warehouse = vehicle_route(1);

% 收集所有无人机服务节点 以及 发射/回收点
drone_A_service_nodes = [];
drone_A_launch_recovery = [];
for k = 1:size(drone_A_tasks, 1)
    launch = drone_A_tasks{k,1};
    recovery = drone_A_tasks{k,3};
    S = drone_A_tasks{k,2};
    if iscell(S), S = cell2mat(S); end
    drone_A_service_nodes = [drone_A_service_nodes, S];
    if launch ~= warehouse
        drone_A_launch_recovery = [drone_A_launch_recovery, launch];
    end
    if recovery ~= warehouse
        drone_A_launch_recovery = [drone_A_launch_recovery, recovery];
    end
end

drone_B_service_nodes = [];
drone_B_launch_recovery = [];
for k = 1:size(drone_B_tasks, 1)
    launch = drone_B_tasks{k,1};
    recovery = drone_B_tasks{k,3};
    S = drone_B_tasks{k,2};
    if iscell(S), S = cell2mat(S); end
    drone_B_service_nodes = [drone_B_service_nodes, S];
    if launch ~= warehouse
        drone_B_launch_recovery = [drone_B_launch_recovery, launch];
    end
    if recovery ~= warehouse
        drone_B_launch_recovery = [drone_B_launch_recovery, recovery];
    end
end

all_drone_service = [drone_A_service_nodes, drone_B_service_nodes];
all_launch_recovery = unique([drone_A_launch_recovery, drone_B_launch_recovery]);

% 车辆路径中的内部节点
vr_inner = vehicle_route(2:end-1);

% 车辆专用服务点: 在车辆路径中但不是无人机服务节点, 也不是发射/回收点 (纯车辆服务)
vehicle_only = setdiff(vr_inner, all_drone_service);

% 发射/回收点 (必须是车辆服务点, 且未被无人机服务)
launch_recovery_vehicle = intersect(all_launch_recovery, vr_inner);

% --- 绘制仓库 (红色五角星) ---
scatter(coords(warehouse,1), coords(warehouse,2), 200, 'r', 'p', ...
    'filled', 'MarkerEdgeColor', 'k');
text(coords(warehouse,1)+0.5, coords(warehouse,2)+1.0, 'Depot(1)', ...
    'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');

% --- 绘制车辆专用服务点 (蓝色圆点) ---
for i = 1:length(vehicle_only)
    nd = vehicle_only(i);
    scatter(coords(nd,1), coords(nd,2), 80, 'b', 'o', 'filled');
    text(coords(nd,1)+0.5, coords(nd,2)-0.5, sprintf('%d', nd), ...
        'FontSize', 8, 'Color', 'b');
end

% --- 绘制发射/回收点 (黑色三角, 上三角=发射, 下三角=回收) ---
% 先统一用黑边圆圈标注
for i = 1:length(launch_recovery_vehicle)
    nd = launch_recovery_vehicle(i);
    scatter(coords(nd,1), coords(nd,2), 120, 'k', 's', 'LineWidth', 1.5);
    text(coords(nd,1)+0.5, coords(nd,2)-0.5, sprintf('%d', nd), ...
        'FontSize', 8, 'Color', 'k');
end

% --- 绘制无人机A服务节点 (红色菱形) ---
for i = 1:length(drone_A_service_nodes)
    nd = drone_A_service_nodes(i);
    scatter(coords(nd,1), coords(nd,2), 100, 'r', 'd', 'filled');
    text(coords(nd,1)+0.5, coords(nd,2)+0.5, sprintf('%d', nd), ...
        'FontSize', 8, 'Color', 'r', 'FontWeight', 'bold');
end

% --- 绘制无人机B服务节点 (绿色方形) ---
for i = 1:length(drone_B_service_nodes)
    nd = drone_B_service_nodes(i);
    scatter(coords(nd,1), coords(nd,2), 100, 'g', 's', 'filled');
    text(coords(nd,1)+0.5, coords(nd,2)+0.5, sprintf('%d', nd), ...
        'FontSize', 8, 'Color', [0 0.5 0], 'FontWeight', 'bold');
end

% --- 车辆路径 (黑色实线) ---
for i = 1:length(vehicle_route)-1
    from_node = vehicle_route(i);
    to_node = vehicle_route(i+1);
    plot([coords(from_node,1), coords(to_node,1)], ...
         [coords(from_node,2), coords(to_node,2)], ...
         'k-', 'LineWidth', 2.5);
    % 箭头
    mid_x = (coords(from_node,1) + coords(to_node,1)) / 2;
    mid_y = (coords(from_node,2) + coords(to_node,2)) / 2;
    dx = coords(to_node,1) - coords(from_node,1);
    dy = coords(to_node,2) - coords(from_node,2);
    len = sqrt(dx^2 + dy^2);
    if len > 0
        quiver(mid_x - dx*0.03, mid_y - dy*0.03, dx*0.06, dy*0.06, ...
            'AutoScale', 'off', 'Color', 'k', 'LineWidth', 1, 'MaxHeadSize', 0.4);
    end
end

% --- 无人机A路径 (红色虚线): 发射点 → 服务点 → 回收点 ---
for k = 1:size(drone_A_tasks, 1)
    launch = drone_A_tasks{k,1};
    S = drone_A_tasks{k,2};
    if iscell(S), S = cell2mat(S); end
    recovery = drone_A_tasks{k,3};

    % 发射点 → 第一个服务点
    plot([coords(launch,1), coords(S(1),1)], ...
         [coords(launch,2), coords(S(1),2)], ...
         'r--', 'LineWidth', 1.8);
    % 标注发射点箭头
    mid_x = mean([coords(launch,1), coords(S(1),1)]);
    mid_y = mean([coords(launch,2), coords(S(1),2)]);
    dx = coords(S(1),1) - coords(launch,1);
    dy = coords(S(1),2) - coords(launch,2);
    len = sqrt(dx^2+dy^2);
    if len > 0
        quiver(mid_x - dx*0.04, mid_y - dy*0.04, dx*0.08, dy*0.08, ...
            'AutoScale', 'off', 'Color', 'r', 'LineWidth', 1, 'MaxHeadSize', 0.5);
    end

    % 服务点之间
    for s = 1:length(S)-1
        plot([coords(S(s),1), coords(S(s+1),1)], ...
             [coords(S(s),2), coords(S(s+1),2)], ...
             'r--', 'LineWidth', 1.8);
    end
    % 最后一个服务点 → 回收点
    plot([coords(S(end),1), coords(recovery,1)], ...
         [coords(S(end),2), coords(recovery,2)], ...
         'r--', 'LineWidth', 1.8);

end

% --- 无人机B路径 (绿色虚线): 发射点 → 服务点 → 回收点 ---
for k = 1:size(drone_B_tasks, 1)
    launch = drone_B_tasks{k,1};
    S = drone_B_tasks{k,2};
    if iscell(S), S = cell2mat(S); end
    recovery = drone_B_tasks{k,3};

    % 发射点 → 第一个服务点
    plot([coords(launch,1), coords(S(1),1)], ...
         [coords(launch,2), coords(S(1),2)], ...
         'g--', 'LineWidth', 1.8);
    mid_x = mean([coords(launch,1), coords(S(1),1)]);
    mid_y = mean([coords(launch,2), coords(S(1),2)]);
    dx = coords(S(1),1) - coords(launch,1);
    dy = coords(S(1),2) - coords(launch,2);
    len = sqrt(dx^2+dy^2);
    if len > 0
        quiver(mid_x - dx*0.04, mid_y - dy*0.04, dx*0.08, dy*0.08, ...
            'AutoScale', 'off', 'Color', [0 0.5 0], 'LineWidth', 1, 'MaxHeadSize', 0.5);
    end

    % 服务点之间
    for s = 1:length(S)-1
        plot([coords(S(s),1), coords(S(s+1),1)], ...
             [coords(S(s),2), coords(S(s+1),2)], ...
             'g--', 'LineWidth', 1.8);
    end
    % 最后一个服务点 → 回收点
    plot([coords(S(end),1), coords(recovery,1)], ...
         [coords(S(end),2), coords(recovery,2)], ...
         'g--', 'LineWidth', 1.8);

end

% --- 图例 ---
h1 = plot(NaN, NaN, 'k-', 'LineWidth', 2.5);                    % 车辆路径
h2 = scatter(NaN, NaN, 80, 'b', 'o', 'filled');                % 车辆服务点
h3 = scatter(NaN, NaN, 120, 'k', 's', 'LineWidth', 1.5);       % 发射/回收点
h4 = scatter(NaN, NaN, 100, 'r', 'd', 'filled');               % 无人机A服务点
h5 = plot(NaN, NaN, 'r--', 'LineWidth', 1.8);                  % 无人机A路径
h6 = scatter(NaN, NaN, 100, 'g', 's', 'filled');               % 无人机B服务点
h7 = plot(NaN, NaN, 'g--', 'LineWidth', 1.8);                  % 无人机B路径
h8 = scatter(NaN, NaN, 200, 'r', 'p', 'filled', 'MarkerEdgeColor', 'k'); % 仓库

legend([h1, h2, h3, h4, h5, h6, h7, h8], ...
    {'车辆路径', '车辆服务点', '发射/回收点', ...
     '无人机A服务点', '无人机A飞行路径', ...
     '无人机B服务点', '无人机B飞行路径', '仓库'}, ...
    'Location', 'bestoutside');

title(title_str, number);
xlabel('X坐标 (km)');
ylabel('Y坐标 (km)');
grid on;
axis equal;
hold off;

end