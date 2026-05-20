%% =========================================================================
clear; clc; close all;
yalmip('clear');

case_name = 'c1';  % 可选: c1 / c2 / c3
fprintf('算例: %s\n', case_name);

%% ==================== 1. 数据加载 ====================
coord_c1 = [
    20.71, 65.86, 0;
    39.00,  5.00, 60;
    1.56, 41.11, 90;
    22.00, 75.00, 30;
    61.00, 67.50, 20;
    45.80, 70.00, 60;
    29.25, 48.50, 40;
    38.67, 14.17, 60;
    32.33, 26.67, 60;
    21.71, 82.14, 70
    ];

coord_c2 = [
    40.51, 48.30,  0;
    31.56, 41.11, 50;
    34.73, 72.99, 30;
    21.22, 13.68, 80;
    28.35, 26.82, 90;
    49.03, 82.29, 70;
    63.96, 43.06, 40;
    38.08, 30.04, 20;
    58.80, 79.50, 50;
    99.51, 38.11, 30
    ];

coord_c3 = [
    20.71, 65.86,  0;
    39.00,  5.00, 90;
    70.34, 30.05, 40;
    22.00, 75.00, 40;
    61.00, 67.50, 60;
    45.80, 70.00, 30;
    92.01, 87.32, 80;
    29.25, 48.50, 20;
    32.33, 26.67, 90;
    21.71, 82.14, 30
    ];

switch case_name
    case 'c1', coord_points = coord_c1;
    case 'c2', coord_points = coord_c2;
    case 'c3', coord_points = coord_c3;
end

coord_xy = coord_points(:, 1:2);
dist_mat = squareform(pdist(coord_xy));  % 欧氏距离矩阵
N = size(coord_points, 1);               % 节点总数
demand  = coord_points(:, 3);            % 各节点需求量
D = 2;                                    % 无人机数量
%% ==================== 2. 参数设置====================
V_t = 50;            % 车辆速度 (km/h)
VA  = 0.8 * V_t;     % 无人机A速度 = 40 km/h
VB  = 1.2 * V_t;     % 无人机B速度 = 60 km/h
V_drone = [VA, VB];

UA = 120;            % 无人机A续航里程 (km)
UB = 80;             % 无人机B续航里程 (km)
U_range = [UA, UB];

WA = 90;             % 无人机A载重 (kg)
WB = 70;             % 无人机B载重 (kg)
W_cap = [WA, WB];

VT = 0;           % 车辆单点服务时间,小时
DT = 0;           % 无人机单点服务时间

% 行驶时间矩阵
tT_mat = dist_mat / V_t;              % 车辆行驶时间
tD_A_mat = dist_mat / VA;             % 无人机A飞行时间
tD_B_mat = dist_mat / VB;             % 无人机B飞行时间
tD_ij_d = cat(3, tD_A_mat, tD_B_mat);  % 无人机飞行时间矩阵 (N×N×D)
Delta_t_max = 0.1;      % 1h等待窗口
M       = 20;           % 大M常数
M_mtz   = N;            % MTZ专用M


%% ==================== 3. 决策变量定义 ====================
% --- 0-1变量 ---
y_ij   = binvar(N, N, 'full');       % y_ij: 车辆从i行驶到j
x_ij_d = binvar(N, N, D, 'full');    % x_ij^d: 无人机d从i飞行到j
q_i    = binvar(N, 1, 'full');
p_i_d  = binvar(N, D, 'full');       % p_i^d: 节点i由无人机d服务
s_i_d  = binvar(N, D, 'full');       % s_i^d: 节点i是无人机d的发射点
h_i_d  = binvar(N, D, 'full');       % h_i^d: 节点i是无人机d的回收点

% --- 连续变量 ---
tT_arr   = sdpvar(N, 1);              % 车辆到达节点i的时间
tT_dep   = sdpvar(N, 1);              % 车辆离开节点i的时间
tD_arr_d = sdpvar(N, D);              % 无人机d到达节点i的时间
tD_dep_d = sdpvar(N, D);              % 无人机d离开节点i的时间
LD       = sdpvar(N, D, 'full');      % 无人机d从节点i发射后的累计飞行时间

%% ==================== 4. 约束条件构建 ====================
cons = [];

% ========== 4.1 车辆路径约束 ==========

% (4.7) 车辆从仓库(节点1)出发恰好一次
cons = [cons, sum(y_ij(1, 2:end)) == 1];

% (4.8) 车辆返回仓库恰好一次
cons = [cons, sum(y_ij(2:end, 1)) == 1];

% 车辆不自环
for i = 1:N
    cons = [cons, y_ij(i, i) == 0];
end

% (4.9) 车辆流量守恒: 对任意中间节点, 入度 = 出度
for i = 2:N
    cons = [cons, sum(y_ij(i, :)) == sum(y_ij(:, i))];

    %% 新增
    cons = [cons, sum(y_ij(i, :)) <= 1];  % 每个节点最多被车辆访问一次
    %% 车辆服务点必须有入边和出边
    cons = [cons, sum(y_ij(i, :)) >= q_i(i)];
    cons = [cons, sum(y_ij(:, i)) >= q_i(i)];
end

% (4.10) 车辆到达时间递推: tT_arr(j) ≥ tT_dep(i) + tT_ij - M(1-y_ij)
for i = 1:N
    for j = 1:N
        if i ~= j
            cons = [cons, ...
                tT_arr(j) >= tT_dep(i) + tT_mat(i, j) - M * (1 - y_ij(i, j))];
        end
    end
end
% (4.11) 无人机离开时间 ≥ 到达时间 + 服务时间
for d=1:D
    for i = 1:N
        % 发射点等待约束: 无人机起飞时间 - 车辆到达时间 ≤ Δt_max + M*(1-s)
        cons = [cons, tD_dep_d(i, d) - tT_arr(i) <= Delta_t_max + M * (1 - s_i_d(i, d))];
        % 回收点等待约束: 车辆离开时间 - 无人机到达时间 ≤ Δt_max + M*(1-h)
        cons = [cons, tT_dep(i) - tD_arr_d(i, d) <= Delta_t_max + M * (1 - h_i_d(i, d))];
        % 同一点内 max(tT_dep-tT_arr, tD_dep_d-tD_arr_d) <= Δt_max + M*(1-s-h)
        cons = [cons, tT_dep(i) - tT_arr(i) <= Delta_t_max + M * (1 - s_i_d(i, d) - h_i_d(i, d))];
        cons = [cons, tD_dep_d(i, d) - tD_arr_d(i, d) <= Delta_t_max + M * (1 - s_i_d(i, d) - h_i_d(i, d))];
    end
end
for d=1:D
    % (4.12)(4.13) 无人机到达时间递推: 双边界约束
    for i = 1:N
        for j = 1:N
            if i ~= j

                % (4.12) 下界: tD_arr(j) ≥ tD_dep(i) + t_ij
                cons = [cons, ...
                    tD_arr_d(j, d) >= tD_dep_d(i, d) + tD_ij_d(i, j, d) - M * (1 - x_ij_d(i, j, d))];
                % (4.13) 上界: tD_arr(j) ≤ tD_dep(i) + t_ij
                cons = [cons, ...
                    tD_arr_d(j, d) <= tD_dep_d(i, d) + tD_ij_d(i, j, d) + M * (1 - x_ij_d(i, j, d))];
            end
        end
    end
end
% (4.14) 车辆路径次序约束 (用MTZ等价替代论文的 o_ij 变量, 消除子回路)
u = sdpvar(N, 1);  % MTZ order variables
cons = [cons, u(1) == 1];
cons = [cons, (1 <= u)&(u <= N)];
for i = 1:N
    for j = 2:N
        if i ~= j
            cons = [cons, u(j) >= u(i) + 1 - M_mtz * (1 - y_ij(i, j))];
        end
    end
end
% (4.15) 任意两点间配送路径唯一性: 每对(i,j)最多由一种工具直连
for i = 1:N
    for j = 1:N
        if i ~= j
            cons = [cons, y_ij(i, j) + x_ij_d(i, j, 1) + x_ij_d(i, j, 2) <= 1];
        end
    end
end
for d=1:D
    % (4.16)
    %  Σ_j x_ij^d ≥ s_i^d + q_i - 1
    % 若有无人机出边, 则必须i是发射点且车辆服务点
    for i = 1:N
        cons = [cons, sum(x_ij_d(i, :, d)) >= s_i_d(i, d) + q_i(i) - 1];
    end
    % (4.17)
    % Σ_i x_ij^d ≥ h_j^d + q_j - 1
    % 若有无人机入边, 则必须j是回收点且车辆服务点
    for j = 1:N
        cons = [cons, sum(x_ij_d(:, j, d)) >= h_i_d(j, d) + q_i(j) - 1];
    end
    % (4.18) 发射点→服务点: 若i是发射点且j是服务点, 则i→j边存在
    for i = 1:N
        for j = 1:N
            if i ~= j
                cons = [cons, x_ij_d(i, j, d) >= s_i_d(i, d) + p_i_d(j, d) - 1];
            end
        end
    end

    % (4.19) 服务点→回收点: 若i是服务点且j是回收点, 则i→j边存在
    for i = 1:N
        for j = 1:N
            if i ~= j
                cons = [cons, x_ij_d(i, j, d) >= p_i_d(i, d) + h_i_d(j, d) - 1];
            end
        end
    end

    % (4.20) 无人机飞行边限制
    for i = 1:N
        for j = 1:N
            if i ~= j
                cons = [cons, x_ij_d(i, j, d) <= p_i_d(i, d) + p_i_d(j, d)];
            end
        end
    end
end
% ========== 4.2 服务分配约束 ==========

% (4.21) 每个节点恰好被车辆或某一架无人机服务一次
% 仓库(节点1)特殊处理: 由车辆"服务"(实际是起降平台/出发点)
cons = [cons, q_i(1) == 1];            % 仓库必须由车辆服务
cons = [cons, p_i_d(1, :) == 0];       % 仓库不由无人机服务
for i = 2:N
    cons = [cons, q_i(i) + sum(p_i_d(i, :)) == 1];
end

for d=1:D
    % (4.22) 同一节点不能既是发射点又是回收点 (同一架无人机)
    for i = 1:N
        cons = [cons, s_i_d(i, d) + h_i_d(i, d) <= 1];
    end
    % (4.23) 发射点数 = 回收点数
    cons = [cons, sum(s_i_d(:, d)) == sum(h_i_d(:, d))];
    % for d = 1:D
    %     % 如果存在无人机服务节点，则必须至少有一个发射点
    %     for i = 1:N
    %         cons = [cons, p_i_d(i, d) >= sum(s_i_d(:, d))];
    %     end
    % end

    % (4.24) 发射点数 ≤ 服务点数
    cons = [cons, sum(s_i_d(:, d)) <= sum(p_i_d(:, d))];

end
%% 新增
for d = 1:D
    % 如果存在无人机服务节点，则发射点和回收点至少各1个
    has_service = sum(p_i_d(:, d));
    cons = [cons, sum(s_i_d(:, d)) >= has_service / N];
    cons = [cons, sum(h_i_d(:, d)) >= has_service / N];
end

% (4.25) 车辆路径与服务点关联: 若y_ij=1 则 q_i=1 且 q_j=1
for i=1:N
    for j=1:N
        cons = [cons, y_ij(i,j) <= q_i(i)];
        cons = [cons, y_ij(i,j) <= q_i(j)];
    end
end


% % (4.26) 路径存在性: 每个非仓库节点至少有一条入边
% for j = 2:N
%     cons = [cons, ...
%         sum(y_ij(:, j)) + sum(x_ij_d(:, j, 1)) + sum(x_ij_d(:, j, 2)) >= 1];
% end

% ========== 4.3 无人机约束 (逐架) ==========
for d = 1:D
    % 无人机不自环
    for i = 1:N
        cons = [cons, x_ij_d(i, i, d) == 0];
    end

    % (4.27)(4.28) 发射/回收点必须是车辆服务点
    cons = [cons, s_i_d(:, d) <= q_i];
    cons = [cons, h_i_d(:, d) <= q_i];

    % % (4.29)
    % for i = 1:N
    %     for j = 1:N
    %         if i == j, continue; end
    %         for k = 1:N
    %             if j == k || i == k, continue; end
    %             cons = [cons, ...
    %                 LD(i, d) >= tD_ij_d(i,j,d) + tD_ij_d(j,k,d) ...
    %                 - M * ( (1 - x_ij_d(i,j,d)) + (1 - x_ij_d(j,k,d)) + (1 - s_i_d(i,d)) ) ];
    %         end
    %     end
    % end

    % % (4.30)/(4.31) 续航里程限制
    % cons = [cons, LD(:, d) <= U_range(d)];

    %% 新增续航约束
    for k = 1:D
    total_dist = 0;
    for i = 1:N
        for j = 1:N
            total_dist = total_dist + dist_mat(i, j) * x_ij_d(i, j, k);
        end
    end
    cons = [cons, total_dist <= U_range(k)];
    end

    %% (4.32)/(4.33) 载重限制: Σ(p_i^d * demand_i) ≤ W_cap(d)
    cons = [cons, p_i_d(:, d)' * demand <= W_cap(d)];

end

% ========== 时间相关约束 ==========
%% 新增
% 车辆出发时间 ≥ 到达时间 + 服务时间 (仅对车辆服务节点)
for i = 2:N
    cons = [cons, tT_dep(i) == tT_arr(i) + VT * q_i(i)];
end
%仓库出发时间为0
cons = [cons, tT_dep(1) == 0];

% % 无人机出发时间 ≥ 到达时间 + 服务时间 (仅对无人机服务节点)
% for d = 1:D
%     for i = 2:N
%         cons = [cons, tD_dep_d(i, d) >= tD_arr_d(i, d) + DT * p_i_d(i, d)];
%     end
% end

% 时间变量非负 (4.35)(4.36)
cons = [cons, tT_arr >= 0, tT_dep >= 0];
cons = [cons, tD_arr_d(:) >= 0, tD_dep_d(:) >= 0];

% 飞行距离非负 (4.39)
cons = [cons, LD(:) >= 0];
%% 新增
% 未访问节点时间清零
for i = 1:N
    % 车辆未服务节点 → 车辆时间=0
    cons = [cons, tT_arr(i) <= M * q_i(i)];
    cons = [cons, tT_dep(i) <= M * q_i(i)];
    % 无人机d未服务/发射/回收节点 → 无人机时间=0
    for d = 1:D
        cons = [cons, tD_arr_d(i, d) <= M * (p_i_d(i, d) + s_i_d(i, d) + h_i_d(i, d))];
        cons = [cons, tD_dep_d(i, d) <= M * (p_i_d(i, d) + s_i_d(i, d) + h_i_d(i, d))];
    end
end
%% 无人机时间与车辆时间同步
for d = 1:D
    for i = 1:N
        % 发射点：无人机到达时间 == 车辆到达时间
        cons = [cons, tD_arr_d(i, d) >= tT_arr(i) - M * (1 - s_i_d(i, d))];
        cons = [cons, tD_arr_d(i, d) <= tT_arr(i) + M * (1 - s_i_d(i, d))];
        
        % 发射点：无人机离开时间 == 车辆离开时间
        cons = [cons, tD_dep_d(i, d) >= tT_dep(i) - M * (1 - s_i_d(i, d))];
        cons = [cons, tD_dep_d(i, d) <= tT_dep(i) + M * (1 - s_i_d(i, d))];
        
        % 回收点：无人机到达时间 == 车辆到达时间
        cons = [cons, tD_arr_d(i, d) >= tT_arr(i) - M * (1 - h_i_d(i, d))];
        cons = [cons, tD_arr_d(i, d) <= tT_arr(i) + M * (1 - h_i_d(i, d))];
    end
end
%% 无人机离开时间与车辆离开时间相等
for d = 1:D
    for i = 1:N
        % 无人机起飞时间 == 车辆离开时间
        cons = [cons, tD_dep_d(i, d) >= tT_dep(i) - M * (1 - s_i_d(i, d))];
        cons = [cons, tD_dep_d(i, d) <= tT_dep(i) + M * (1 - s_i_d(i, d))];
    end
end
for d = 1:D
    for i = 1:N
        % 无人机到达时间 == 车辆到达时间
        cons = [cons, tD_arr_d(i, d) >= tT_arr(i) - M * (1 - h_i_d(i, d))];
        cons = [cons, tD_arr_d(i, d) <= tT_arr(i) + M * (1 - h_i_d(i, d))];
    end
end
%% 新增
%无人机网络流约束
for d = 1:D
    for i = 1:N
        outflow = sum(x_ij_d(i, :, d));
        inflow  = sum(x_ij_d(:, i, d));
        
        % 流量守恒：发射点净出1，回收点净入1，中间点净0
        cons = [cons, outflow - inflow == s_i_d(i, d) - h_i_d(i, d)];
        
        % 每个节点最多一条出边、一条入边
        cons = [cons, outflow <= 1, inflow <= 1];
        
        % 强制普通服务节点（p=1且s=0且h=0）必须有出边和入边
        cons = [cons, outflow >= p_i_d(i, d) - s_i_d(i, d) - h_i_d(i, d)];
        cons = [cons, inflow  >= p_i_d(i, d) - s_i_d(i, d) - h_i_d(i, d)];
    end
end

%%目标函数
Objective = tT_arr(1);


%%求解
fprintf('节点数: %d, 无人机数: %d\n', N, D);
fprintf('约束总数: %d\n', length(cons));
fprintf('目标函数: Σ t_j^{T+} (车辆到达各投放点时间之和)\n');

ops = sdpsettings('solver', 'gurobi', 'verbose', 1, ...
    'gurobi.TimeLimit', 5000000, 'gurobi.MIPGap', 1e-4, ...
    'gurobi.LPWarmStart', 0, 'gurobi.MIPFocus', 1, ...
    'gurobi.Cuts', 3, 'gurobi.Heuristics', 0.5);

fprintf('开始求解...\n');
tic;
sol = optimize(cons, Objective, ops);
elapsed = toc;

% 如果模型不可行 (code 12)，计算 IIS 进行诊断
if sol.problem == 12
    fprintf('\n模型不可行，计算 IIS...\n');
    try
        [model,~] = export(cons, Objective, ops);
        iis = gurobi_iis(model);
        conflict_rows = find(iis.Arows);
        fprintf('冲突约束行索引: %s\n', mat2str(conflict_rows));
        gurobi_write(model, 'yalmiptest.lp');
        fid = fopen('yalmiptest.lp','r');
        lines = textscan(fid,'%s','Delimiter','\n');
        fclose(fid);
        lp_lines = lines{1};
        fprintf('\n===== 冲突约束内容 =====\n');
        for r = conflict_rows'
            idx = find(contains(lp_lines, sprintf('R%d:', r-1)));
            if ~isempty(idx)
                fprintf('行%d → %s\n', r, lp_lines{idx});
            end
        end
    catch ME
        fprintf('IIS 计算失败: %s\n', ME.message);
    end
end

%%结果提取
fprintf('\n求解状态: %s (code %d)\n', sol.info, sol.problem);
fprintf('求解耗时: %.2f 秒\n', elapsed);

if sol.problem == 0 || sol.problem == 3
    if sol.problem == 3
        fprintf('\n注意: 求解达到时间限制，以下为当前最优解\n');
    end
    y_ij_val    = round(value(y_ij));
    x_ij_d_val  = round(value(x_ij_d));
    q_i_val    = round(value(q_i));
    p_i_val    = round(value(p_i_d));
    s_i_val    = round(value(s_i_d));
    h_i_val    = round(value(h_i_d));
    tT_arr_value = value(tT_arr);
    tT_dep_value = value(tT_dep);
    tD_arr_value = value(tD_arr_d);
    tD_dep_value = value(tD_dep_d);
    LD_value     = value(LD);
    u_value    = value(u);
    obj_value  = value(Objective);

    fprintf('目标函数值 (Σ t_j^{T+}) = %.4f\n', obj_value);

    %% 显示服务分配
    fprintf('\n===== 服务分配 =====\n');
    for i = 1:N
        svc = '';
        if q_i_val(i) == 1
            if i == 1, svc = [svc, '仓库 '];
            else,      svc = [svc, '车辆 ']; end
        end
        for d = 1:D
            if p_i_val(i, d) == 1
                svc = [svc, sprintf('无人机%s ', char('A'+d-1))];
            end
            if s_i_val(i, d) == 1
                svc = [svc, sprintf('[发射%s] ', char('A'+d-1))];
            end
            if h_i_val(i, d) == 1
                svc = [svc, sprintf('[回收%s] ', char('A'+d-1))];
            end
        end
        fprintf('  节点%2d (需求%3dkg): %s\n', i, demand(i), svc);
    end

    %% 提取车辆路径
    curr = 1;
    vehicle_route = 1;
    visited_vehicle = false(1, N);
    visited_vehicle(1) = true;
    while true
        nxt = find(y_ij_val(curr, :) == 1);
        if isempty(nxt) || nxt(1) == 1
            break;
        end
        nxt = nxt(1);
        vehicle_route = [vehicle_route, nxt];
        visited_vehicle(nxt) = true;
        curr = nxt;
    end
    vehicle_route = [vehicle_route, 1];

    fprintf('\n===== 车辆路径 =====\n');
    fprintf('  ');
    fprintf('%d ', vehicle_route);
    fprintf('\n  车辆访问节点数: %d (含仓库)\n', length(vehicle_route)-1);

    % 计算车辆路径总距离和时间
    vehicle_total_dist = 0;
    for k = 1:length(vehicle_route)-1
        vehicle_total_dist = vehicle_total_dist + dist_mat(vehicle_route(k), vehicle_route(k+1));
    end
    fprintf('  车辆总行驶距离: %.1f km\n', vehicle_total_dist);

    %% 提取各架无人机路径
    drone_trips = cell(1, D);
    for d = 1:D
        xv = x_ij_d_val(:, :, d);
        sv = s_i_val(:, d);
        hv = h_i_val(:, d);
        launch_nodes = find(sv == 1);

        trips_this_drone = {};
        for lp = 1:length(launch_nodes)
            start_node = launch_nodes(lp);
            curr_node  = start_node;
            trip_nodes = [start_node];

            while true
                nxt = find(xv(curr_node, :) == 1);
                if isempty(nxt), break; end
                nxt = nxt(1);
                trip_nodes = [trip_nodes, nxt];
                if hv(nxt) == 1, break; end  % 到达回收点
                curr_node = nxt;
            end
            trips_this_drone{end+1} = trip_nodes;
        end
        drone_trips{d} = trips_this_drone;

        fprintf('\n===== 无人机 %s 路径 =====\n', char('A' + d - 1));
        if isempty(trips_this_drone)
            fprintf('  (无任务)\n');
        else
            for t = 1:length(trips_this_drone)
                tr = trips_this_drone{t};
                fprintf('  起降 %d: ', t);
                fprintf('%d ', tr);
                % 计算飞行距离
                d_sum = 0;
                for k = 1:length(tr)-1
                    d_sum = d_sum + dist_mat(tr(k), tr(k+1));
                end
                fprintf('  [飞行%.1fkm / 续航%dkm]', d_sum, U_range(d));
                % 检查载重
                service_nodes = tr(2:end-1);  % 排除发射/回收点
                if ~isempty(service_nodes)
                    load_sum = sum(demand(service_nodes));
                    fprintf(' [载重%dkg / 容量%dkg]', load_sum, W_cap(d));
                end
                fprintf('\n');
            end
        end
    end

    %% 时间详情
    fprintf('\n===== 节点到达/离开时间 (小时) =====\n');
    fprintf('节点 | 车辆次序 | 车辆到达 | 车辆离开 | 无人机A到达 | 无人机A离开 | 无人机B到达 | 无人机B离开\n');
    fprintf('-----+----------+----------+----------+------------+------------+------------+------------\n');
    for i = 1:N
        fprintf(' %2d  | %8.1f | %8.3f | %8.3f | %10.3f | %10.3f | %10.3f | %10.3f\n', ...
            i, u_value(i), tT_arr_value(i), tT_dep_value(i), ...
            tD_arr_value(i,1), tD_dep_value(i,1), ...
            tD_arr_value(i,2), tD_dep_value(i,2));
    end

    %% ==================== 8. 绘制路径图 ====================
    figure('Name', ['VHD-TSP 配送路径 - ' case_name], ...
        'Position', [100, 100, 900, 750]);
    hold on; axis equal;
    xc = coord_xy(:, 1);
    yc = coord_xy(:, 2);

    % 绘制所有节点
    scatter(xc, yc, 100, 'k', 'filled', 'MarkerEdgeColor', 'w', 'LineWidth', 1);
    for i = 1:N
        if i == 1
            text(xc(i) + 1.5, yc(i) + 1.5, ['仓库(', num2str(i), ')'], ...
                'FontSize', 10, 'BackgroundColor', 'y', 'FontWeight', 'bold');
        else
            text(xc(i) + 1.5, yc(i) + 1.5, num2str(i), ...
                'FontSize', 10, 'BackgroundColor', 'w', 'FontWeight', 'bold');
        end
    end

    % 高亮仓库
    plot(xc(1), yc(1), 'ks', 'MarkerSize', 18, ...
        'MarkerFaceColor', 'y', 'LineWidth', 2);

    % 绘制车辆路径 (蓝色实线)
    for k = 1:length(vehicle_route) - 1
        i = vehicle_route(k);
        j = vehicle_route(k + 1);
        plot([xc(i), xc(j)], [yc(i), yc(j)], 'b-', 'LineWidth', 2.5);
        % 箭头
        dx = xc(j) - xc(i);
        dy = yc(j) - yc(i);
        mid_x = xc(i) + 0.5 * dx;
        mid_y = yc(i) + 0.5 * dy;
        quiver(mid_x, mid_y, dx/10, dy/10, 'b', ...
            'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
    end

    % 绘制无人机路径
    drone_colors = {'r', [1, 0.4, 0]};   % A:红色, B:橙色
    drone_names  = {'无人机A', '无人机B'};
    drone_style  = {'--', '-.'};
    for d = 1:D
        trips = drone_trips{d};
        if ~isempty(trips)
            for t = 1:length(trips)
                tr = trips{t};
                if length(tr) >= 2
                    for k = 1:length(tr) - 1
                        i = tr(k);
                        j = tr(k + 1);
                        plot([xc(i), xc(j)], [yc(i), yc(j)], ...
                            drone_style{d}, 'Color', drone_colors{d}, 'LineWidth', 2);
                        dx = xc(j) - xc(i);
                        dy = yc(j) - yc(i);
                        mid_x = xc(i) + 0.5 * dx;
                        mid_y = yc(i) + 0.5 * dy;
                        quiver(mid_x, mid_y, dx/10, dy/10, ...
                            'Color', drone_colors{d}, 'LineWidth', 1.5, ...
                            'MaxHeadSize', 0.5, 'AutoScale', 'off');
                    end
                end
            end
        end
    end

    xlabel('X坐标 (km)', 'FontSize', 12);
    ylabel('Y坐标 (km)', 'FontSize', 12);
    title(sprintf('VHD-TSP 最优配送路径 (%s算例, obj = %.3f h)', ...
        case_name, obj_value), 'FontSize', 13);
    legend('投放点', '车辆路径', '无人机A路径', '无人机B路径', '仓库', ...
        'Location', 'best');
    grid on;
    hold off;

    % % 保存图片
    % saveas(gcf, sprintf('VHD_TSP_%s_result.png', case_name));
    % fprintf('\n路径图已保存: VHD_TSP_%s_result.png\n', case_name);

else
    if sol.problem ~= 3
        fprintf('\n!!!!!!!! 求解失败 !!!!!!!!\n');
        fprintf('状态码: %d, 信息: %s\n', sol.problem, sol.info);
    end
end

fprintf('\n======== 运行结束 ========\n');