%% ===========================================================================
% 改进自适应遗传算法 (AGA-CO) 求解 VHD-TSP 问题
% 基于第五章算法描述
% 算例: c1, c2, c3
% ===========================================================================

clc; clear; close all;

%% ==================== 全局参数设置 ====================
% --- 车辆参数 ---
V_T = 50;           % 车辆速度 (km/h)
service_T = 0.05;   % 车辆在每个投放点的服务时间 (小时), 约3分钟

% --- 异构无人机参数 ---
% 无人机A (重型长续航)
V_A = V_T * 0.8;  % 无人机A速度 = 40 km/h
W_A = 40;           % 无人机A载重能力 (kg)
U_A = 120;          % 无人机A续航里程 (km)
service_D_A = 0.03; % 无人机A服务时间 (小时)

% 无人机B (轻型短途快速)
V_B = V_T * 1.2;  % 无人机B速度 = 60 km/h
W_B = 20;           % 无人机B载重能力 (kg)
U_B = 80;           % 无人机B续航里程 (km)
service_D_B = 0.02; % 无人机B服务时间 (小时)

% --- 协同约束 ---
At_max = 5 / 60;    % 最大相互等待时间 (小时) = 5分钟
M = 1e6;            % 大M常数

% --- 算法参数 ---
pop_size = 100;         % 种群规模
max_gen = 1000;         % 最大迭代次数
stall_limit = 100;      % 连续无改进终止代数
tournament_k = 5;       % 锦标赛选择规模
elite_count = 2;        % 精英保留数量
Pc_max = 0.8;           % 最大交叉概率
alpha = 0.2;            % 交叉概率调整权重

fprintf('============================================\n\n');

%% 获取算例数据
[coords, demands, case_name] = get_crood_data('a1');
n_nodes = size(coords, 1);
warehouse = 1;

% 预计算距离矩阵 (只计算一次)
dist_matrix = zeros(n_nodes, n_nodes);
for i = 1:n_nodes
    for j = 1:n_nodes
        dist_matrix(i,j) = sqrt((coords(i,1)-coords(j,1))^2 + (coords(i,2)-coords(j,2))^2);
    end
end

% 运行AGA-CO算法
[best_chromosome, best_fitness, fitness_history, run_time] = ...
    AGA_CO(coords, demands, dist_matrix, n_nodes, warehouse, ...
    V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
    service_D_A, service_D_B, At_max, M, pop_size, max_gen, ...
    stall_limit, tournament_k, elite_count, Pc_max, alpha);

% 解码最优解
[vehicle_route, drone_tasks_A, drone_tasks_B, total_time, details] = ...
    decode_solution(best_chromosome, dist_matrix, demands, n_nodes, ...
    V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
    service_D_A, service_D_B, At_max, M);

% 输出结果
fprintf('\n--- 最优解结果 ---\n');
fprintf('车辆路径: ');
fprintf('%d -> ', vehicle_route(1:end-1));
fprintf('%d\n', vehicle_route(end));

if ~isempty(drone_tasks_A)
    fprintf('无人机A任务:\n');
    for k = 1:size(drone_tasks_A, 1)
        node_list = drone_tasks_A{k,2};
        if iscell(node_list), node_list = cell2mat(node_list); end
        fprintf('  任务%d: 发射点%d -> 服务点', k, drone_tasks_A{k,1});
        fprintf('[%s] -> 回收点%d\n', num2str(node_list), drone_tasks_A{k,3});
    end
else
    fprintf('无人机A: 无任务\n');
end

if ~isempty(drone_tasks_B)
    fprintf('无人机B任务:\n');
    for k = 1:size(drone_tasks_B, 1)
        node_list = drone_tasks_B{k,2};
        if iscell(node_list), node_list = cell2mat(node_list); end
        fprintf('  任务%d: 发射点%d -> 服务点', k, drone_tasks_B{k,1});
        fprintf('[%s] -> 回收点%d\n', num2str(node_list), drone_tasks_B{k,3});
    end
else
    fprintf('无人机B: 无任务\n');
end

fprintf('\n--- 时间详情 ---\n');
fprintf('车辆到达仓库时间: %.4f 小时 (%.2f 分钟)\n', ...
    details.vehicle_return_time, details.vehicle_return_time*60);
if ~isempty(details.drone_end_times)
    fprintf('无人机任务完成时间: ');
    for i = 1:length(details.drone_end_times)
        fprintf('任务%d=%.2fh ', i, details.drone_end_times(i));
    end
    fprintf('\n');
end
fprintf('总配送时间: %.4f 小时 (%.2f 分钟)\n', total_time, total_time*60);
fprintf('最优适应度: %.6f\n', best_fitness);
fprintf('求解时间: %.2f 秒\n', run_time);

% 打印各节点的车辆到达/离开时间
fprintf('\n--- 各节点车辆到达/离开时间 ---\n');
for i = 1:length(vehicle_route)
    node = vehicle_route(i);
    arr = details.real_arrival(node);
    dep = details.real_depart(node);
    if arr >= 0
        fprintf('  节点%d: 到达时间=%.4fh (%.1fmin)', node, arr, arr*60);
        if dep >= 0
            fprintf(', 离开时间=%.4fh (%.1fmin)', dep, dep*60);
        end
        fprintf('\n');
    end
end

% 验证结果
fprintf('\n--- 约束验证 ---\n');
constraint_ok = verify_solution(vehicle_route, drone_tasks_A, drone_tasks_B, ...
    coords, demands, dist_matrix, V_A, V_B, W_A, W_B, U_A, U_B);
if constraint_ok
    fprintf('所有约束满足！\n');
else
    fprintf('警告：存在约束违反！\n');
end

% --- 绘制结果 ---
figure('Name', sprintf('%s 联合配送路径', case_name), 'Position', [100, 100, 700, 600]);
plot_solution(vehicle_route, drone_tasks_A, drone_tasks_B, coords, ...
    sprintf('%s 联合配送路径规划 (总时间: %.2f h)', case_name, total_time));

% 绘制收敛曲线
figure('Name', sprintf('%s 收敛曲线', case_name), 'Position', [850, 100, 500, 400]);
plot(1:length(fitness_history), fitness_history, 'b-', 'LineWidth', 1.5);
xlabel('迭代次数'); ylabel('适应度值');
title(sprintf('%s AGA-CO收敛曲线', case_name));
grid on;

fprintf('\n');

fprintf('\n============ 所有算例求解完成 ============\n');

%%                    AGA-CO 主算法
function [best_chromosome, best_fitness, fitness_history, run_time] = ...
    AGA_CO(coords, demands, dist_matrix, n_nodes, warehouse, ...
    V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
    service_D_A, service_D_B, At_max, M, pop_size, max_gen, ...
    stall_limit, tournament_k, elite_count, Pc_max, alpha)

tic;

% 初始化种群
population = initialize_population(pop_size, n_nodes, warehouse, coords, ...
    demands, dist_matrix, V_A, V_B, W_A, W_B, U_A, U_B);

% 适应度评估
fitness = zeros(pop_size, 1);
for i = 1:pop_size
    fitness(i) = evaluate_fitness(population{i}, dist_matrix, demands, n_nodes, ...
        V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
        service_D_A, service_D_B, At_max, M);
end

% 记录最优
[best_fitness, best_idx] = max(fitness);
best_chromosome = population{best_idx};

fitness_history = zeros(max_gen, 1);
fitness_history(1) = best_fitness;

stall_count = 0;

%%AGA_CO算法主循环
for gen = 2:max_gen
    % 计算种群多样性
    diversity = compute_diversity(fitness);

    % 自适应交叉概率
    D_max = (max(fitness) - min(fitness))/2;%近似找到D_max
    Pc = Pc_max * exp(-0.005 * gen) + alpha * (diversity / D_max);

    % 自适应变异概率
    if diversity < 0.2
        Pm = 0.3;
    else
        Pm = 0.1 + 0.1 * (gen / max_gen);
    end

    % 生成新一代种群
    new_population = cell(pop_size, 1);

    %% 精英保留
    [~, sort_idx] = sort(fitness, 'descend');
    for e = 1:elite_count
        elite_chrom = population{sort_idx(e)};
        % if length(elite_chrom.vehicle_route) >= 3
        %         elite_chrom = resolve_drone_conflicts(elite_chrom, n_nodes, warehouse);
        %         elite_chrom = validate_and_repair(elite_chrom, n_nodes, warehouse, ...
        %             demands, dist_matrix, V_A, V_B, W_A, W_B, U_A, U_B);
        % end
        new_population{e} = elite_chrom;
    end

     % 生成剩余个体
    for i = elite_count+1:pop_size
        % 锦标赛选择
        parent1 = tournament_select(population, fitness, tournament_k);
        parent2 = tournament_select(population, fitness, tournament_k);

        % 动态协同交叉
        if rand() < Pc
            offspring = dynamic_cooperative_crossover(parent1, parent2, ...
                demands, dist_matrix, V_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
                n_nodes, warehouse);
        else
            offspring = parent1;
         end

         % 路径翻转 (2-opt局部搜索)
        offspring.vehicle_route = two_opt_local_search_fast(offspring.vehicle_route, dist_matrix, warehouse);

        % 约束感知变异
        if rand() < Pm
            offspring = constraint_aware_mutation(offspring, demands, dist_matrix, ...
                V_A, V_B, W_A, W_B, U_A, U_B, n_nodes, warehouse);
            offspring.vehicle_route = two_opt_local_search_fast(offspring.vehicle_route, dist_matrix, warehouse);
        end

        % 完整性校验和修复 (防止交叉/变异引入不可行解)
        offspring = validate_and_repair(offspring, n_nodes, warehouse, ...
            demands, dist_matrix, V_A, V_B, W_A, W_B, U_A, U_B);

        new_population{i} = offspring;
    end

    population = new_population;

    % 适应度评估
    for i = 1:pop_size
        fitness(i) = evaluate_fitness(population{i}, dist_matrix, demands, n_nodes, ...
            V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
            service_D_A, service_D_B, At_max, M);
    end

    % 更新最优
    [current_best_fit, current_best_idx] = max(fitness);
    if current_best_fit > best_fitness
        best_fitness = current_best_fit;
        best_chromosome = population{current_best_idx};
        stall_count = 0;
    else
        stall_count = stall_count + 1;
    end

    fitness_history(gen) = best_fitness;

    % 终止检测
    if stall_count >= stall_limit
        fitness_history = fitness_history(1:gen);
        break;
    end

    % 显示进度
    if mod(gen, 10) == 0 || gen == 2
        fprintf('  迭代 %d/%d, 最优适应度: %.6f, 多样性: %.4f, Pc: %.4f, Pm: %.4f\n', ...
            gen, max_gen, best_fitness, diversity, Pc, Pm);
    end
end

run_time = toc;

% 局部搜索优化最终解 (降低迭代次数以加速)
best_chromosome = local_search_optimize(best_chromosome, demands, dist_matrix, ...
    V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
    service_D_A, service_D_B, At_max, M, ...
    n_nodes, warehouse, 20);

% 重新评估
best_fitness = evaluate_fitness(best_chromosome, dist_matrix, demands, n_nodes, ...
    V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
    service_D_A, service_D_B, At_max, M);

fprintf('  AGA-CO求解完成, 用时: %.2f秒, 最优适应度: %.6f\n', run_time, best_fitness);
end

%%种群初始化
function population = initialize_population(pop_size, n_nodes, warehouse, coords, ...
    demands, dist_matrix, V_A, V_B, W_A, W_B, U_A, U_B)

population = cell(pop_size, 1);%创建一个cell数组来存储种群中的每个个体（染色体）。每个染色体将包含车辆路径和无人机任务分配的信息。

%%测试:绘制随机种群初始化的第一份路径图
greedy_drawn = false;
random_drawn = false;
for i = 1:pop_size
    if i <= pop_size / 2
        % 50% 贪婪生成
        chromosome = generate_greedy_chromosome(n_nodes, warehouse, demands, ...
            dist_matrix, W_A, W_B, U_A, U_B, coords);
        % 对第一个随机个体绘制路径图
        if ~greedy_drawn
            plot_greedy_path(coords, chromosome.vehicle_route, ...
                chromosome.drone_A_tasks, chromosome.drone_B_tasks, ...
                '贪婪生成第1份路径',1);
            greedy_drawn = true;
        end
    else
        % 50% 随机生成
        chromosome = generate_random_chromosome(n_nodes, warehouse, coords, ...
            demands, dist_matrix, V_A, V_B, W_A, W_B, U_A, U_B);
        if ~random_drawn
            plot_greedy_path(coords, chromosome.vehicle_route, ...
                chromosome.drone_A_tasks, chromosome.drone_B_tasks, ...
                '随机生成第1份路径',2);
            random_drawn = true;
        end
    end

    % 完整性校验和修复
    chromosome = validate_and_repair(chromosome, n_nodes, warehouse, ...
        demands, dist_matrix, V_A, V_B, W_A, W_B, U_A, U_B);

    population{i} = chromosome;
end
end

%%贪婪染色体生成
function chromosome = generate_greedy_chromosome(n_nodes, warehouse, demands, ...
    dist_matrix, W_A, W_B, U_A, U_B, coords)

% 车辆路径: 随机排列所有节点 (仓库在首尾)
inner_nodes = 2:n_nodes;
vehicle_order = inner_nodes(randperm(length(inner_nodes)));
vehicle_route = [warehouse, vehicle_order, warehouse];

% 2-opt优化初始路径
vehicle_route = two_opt_local_search_fast(vehicle_route, dist_matrix, warehouse);

% 贪婪分配无人机任务
[drone_A_tasks, drone_B_tasks, vehicle_route] = ...
    greedy_drone_assignment(vehicle_route, demands, dist_matrix, ...
    W_A, W_B, U_A, U_B);

chromosome.vehicle_route = vehicle_route;
chromosome.drone_A_tasks = drone_A_tasks;
chromosome.drone_B_tasks = drone_B_tasks;
fprintf('--- 贪婪生成个体 ---\n');
fprintf('车辆路径: ');
fprintf('%d -> ', vehicle_route(1:end-1));
fprintf('%d\n', vehicle_route(end));

if ~isempty(drone_A_tasks)
    fprintf('无人机A任务 (%d个):\n', size(drone_A_tasks,1));
    for k = 1:size(drone_A_tasks, 1)
        node_list = drone_A_tasks{k,2};
        if iscell(node_list), node_list = cell2mat(node_list); end
        fprintf('  任务%d: 发射点%d -> 服务点[%s] -> 回收点%d\n', ...
            k, drone_A_tasks{k,1}, num2str(node_list), drone_A_tasks{k,3});
    end
else
    fprintf('无人机A: 无任务\n');
end

if ~isempty(drone_B_tasks)
    fprintf('无人机B任务 (%d个):\n', size(drone_B_tasks,1));
    for k = 1:size(drone_B_tasks, 1)
        node_list = drone_B_tasks{k,2};
        if iscell(node_list), node_list = cell2mat(node_list); end
        fprintf('  任务%d: 发射点%d -> 服务点[%s] -> 回收点%d\n', ...
            k, drone_B_tasks{k,1}, num2str(node_list), drone_B_tasks{k,3});
    end
else
    fprintf('无人机B: 无任务\n');
end
end
%%随机染色体生成
function chromosome = generate_random_chromosome(n_nodes, warehouse, coords, ...
    demands, dist_matrix, V_A, V_B, W_A, W_B, U_A, U_B)

% 车辆路径: 随机排列所有节点 (仓库在首尾)
inner_nodes = 2:n_nodes;
vehicle_order = inner_nodes(randperm(length(inner_nodes)));
vehicle_route = [warehouse, vehicle_order, warehouse];

% 2-opt优化初始路径
vehicle_route = two_opt_local_search_fast(vehicle_route, dist_matrix, warehouse);


% 全随机无人机任务分配
[drone_A_tasks, drone_B_tasks, vehicle_route] = ...
    random_drone_assignment(vehicle_route, demands, dist_matrix, ...
    V_A, V_B, W_A, W_B, U_A, U_B);

chromosome.vehicle_route = vehicle_route;
chromosome.drone_A_tasks = drone_A_tasks;
chromosome.drone_B_tasks = drone_B_tasks;
% fprintf('--- 随机生成个体 ---\n');
% fprintf('车辆路径: ');
% fprintf('%d -> ', vehicle_route(1:end-1));
% fprintf('%d\n', vehicle_route(end));

% if ~isempty(drone_A_tasks)
%     fprintf('无人机A任务 (%d个):\n', size(drone_A_tasks,1));
%     for k = 1:size(drone_A_tasks, 1)
%         node_list = drone_A_tasks{k,2};
%         if iscell(node_list), node_list = cell2mat(node_list); end
%         fprintf('  任务%d: 发射点%d -> 服务点[%s] -> 回收点%d\n', ...
%             k, drone_A_tasks{k,1}, num2str(node_list), drone_A_tasks{k,3});
%     end
% else
%     fprintf('无人机A: 无任务\n');
% end

% if ~isempty(drone_B_tasks)
%     fprintf('无人机B任务 (%d个):\n', size(drone_B_tasks,1));
%     for k = 1:size(drone_B_tasks, 1)
%         node_list = drone_B_tasks{k,2};
%         if iscell(node_list), node_list = cell2mat(node_list); end
%         fprintf('  任务%d: 发射点%d -> 服务点[%s] -> 回收点%d\n', ...
%             k, drone_B_tasks{k,1}, num2str(node_list), drone_B_tasks{k,3});
%     end
% else
%     fprintf('无人机B: 无任务\n');
% end
end

%%2-opt快速局部搜索
function route = two_opt_local_search_fast(route, dist_matrix, warehouse)
max_iterations = 1e10;  % 最大迭代次数上限 (防止卡死)
improved = true;
iter_count = 0;
while improved && iter_count < max_iterations
    improved = false;
    iter_count = iter_count + 1;
    best_dist = path_length(route, dist_matrix);
    for i = 2:length(route)-3
        for j = i+1:length(route)-1
            new_route = route;
            new_route(i:j) = fliplr(new_route(i:j));
            new_dist = path_length(new_route, dist_matrix);
            if new_dist < best_dist - 1e-6
                route = new_route;
                best_dist = new_dist;
                improved = true;
            end
        end
    end
end
end

%% ===========================================================================
%%       全量路径初始化 + 贪心筛选分工 + 路径重排 + 配对起降点 (新算法)
%% ===========================================================================
% 算法步骤:
%   1. 初始化全量路径: 随机排列所有节点生成一条初始车辆路径,
%      此时所有节点临时视为车辆服务点。
%   2. 贪心筛选分工: 遍历路径上除仓库外的所有节点,
%      依据"远距大需求给A型无人机、紧急小需求给B型无人机、超大超重留给车"
%      的规则进行筛选。符合条件的节点被剥离出来, 指派给对应无人机。
%   3. 路径重排: 保留在路径上未被剥离的节点, 按原顺序排列,
%      即为最终的车辆服务路径。
%   4. 配对起降点: 针对每个被剥离的无人机服务节点,
%      在其原路径位置向前寻找最近的车辆服务点作为发射点,
%      向后寻找最近的车辆服务点作为回收点。
%% ===========================================================================
function [drone_A_tasks, drone_B_tasks, vehicle_route] = ...
    full_path_drone_assignment(n_nodes, warehouse, demands, dist_matrix, ...
    V_A, V_B, W_A, W_B, U_A, U_B)

% ============================
% 步骤1: 初始化全量路径
% 随机排列所有节点 (除仓库外的节点) 生成一条初始车辆路径,
% 此时所有节点都临时视为车辆服务点。
% 对该路径施加2-opt优化, 确保为无交叉的环形路线。
% ============================
all_inner_nodes = 2:n_nodes;
permuted_nodes = all_inner_nodes(randperm(length(all_inner_nodes)));
initial_path = [warehouse, permuted_nodes, warehouse];
% 2-opt优化初始全量路径, 使其成为无交叉环形
initial_path = two_opt_local_search_fast(initial_path, dist_matrix, warehouse);
n_path_nodes = length(initial_path) - 2;  % 内部节点数量

% 记录每个内部节点在初始路径中的原始位置索引 (1-based, 对应内部节点)
node_pos_in_initial = containers.Map('KeyType', 'double', 'ValueType', 'double');
for pos = 1:n_path_nodes
    node_pos_in_initial(initial_path(pos + 1)) = pos;
end

% ============================
% 步骤2: 贪心筛选分工
% 遍历路径上除仓库外的所有节点, 按 A→B→车 的优先级筛选:
%   对每个节点, 先判断是否适合A型无人机;
%   A型不适合再判断B型; 都不适合则留给车辆。
% ============================

% 计算每个节点到仓库的距离 (用于判断"远距")
dist_to_warehouse = zeros(1, n_nodes);
for i = 1:n_nodes
    dist_to_warehouse(i) = dist_matrix(warehouse, i);
end
median_dist = median(dist_to_warehouse(2:end));  % 中位距离作为远近阈值

% 标记每个内部节点是否被剥离给无人机
is_stripped = false(1, n_path_nodes);
stripped_nodes = struct('node', {}, 'drone_type', {}, 'original_pos', {});

for pos = 1:n_path_nodes
    node = initial_path(pos + 1);

    % 判断距离远近
    is_far = dist_to_warehouse(node) > median_dist;
    demand_val = demands(node);

    % 规则1: 先尝试分配给A型无人机
    % A型条件: 需求量不超过W_A, 且(远距大需求 或 中等需求但A型更适合)
    if demand_val <= W_A && is_far && demand_val > W_B
        % 远距大需求 → A型无人机
        is_stripped(pos) = true;
        stripped_nodes(end+1).node = node;
        stripped_nodes(end).drone_type = 'A';
        stripped_nodes(end).original_pos = pos;
        continue;
    end

    % 规则2: A型不适合, 再判断B型无人机
    % B型条件: 需求量不超过W_B (小需求/紧急需求用快速无人机)
    if demand_val <= W_B
        is_stripped(pos) = true;
        stripped_nodes(end+1).node = node;
        stripped_nodes(end).drone_type = 'B';
        stripped_nodes(end).original_pos = pos;
        continue;
    end

    % 规则3: A和B都不适合 → 留给车辆服务
    % (包括: 需求 > W_A 的超大超重节点, 以及不满足A/B条件的节点)
end

% ============================
% 步骤3: 路径重排
% 保留未被剥离的节点, 按原顺序排列, 即为最终的车辆服务路径。
% ============================
remaining_positions = find(~is_stripped);
remaining_nodes = initial_path(remaining_positions + 1);
vehicle_inner = remaining_nodes;

% ============================
% 步骤4: 配对起降点
% 针对每个被剥离的无人机服务节点,
% 在其原路径位置向前寻找最近的车辆服务点作为发射点,
% 向后寻找最近的车辆服务点作为回收点。
% ============================
drone_A_tasks = {};
drone_B_tasks = {};

% 构建"车辆服务点在初始路径中的位置"列表
vehicle_positions_in_initial = remaining_positions;  % 未被剥离节点的原始位置

for s = 1:length(stripped_nodes)
    sn = stripped_nodes(s);
    node = sn.node;
    drone_type = sn.drone_type;
    original_pos = sn.original_pos;

    % 向前寻找最近的车辆服务点作为发射点
    forward_candidates = vehicle_positions_in_initial(vehicle_positions_in_initial < original_pos);
    if isempty(forward_candidates)
        % 如果前面没有车辆服务点, 使用仓库 (位置0)
        launch_node = warehouse;
    else
        % 选择原路径位置最接近的 (最大的位置索引)
        [~, closest_idx] = max(forward_candidates);
        launch_pos = forward_candidates(closest_idx);
        launch_node = initial_path(launch_pos + 1);
    end

    % 向后寻找最近的车辆服务点作为回收点
    backward_candidates = vehicle_positions_in_initial(vehicle_positions_in_initial > original_pos);
    if isempty(backward_candidates)
        % 如果后面没有车辆服务点, 使用仓库 (位置 n_path_nodes+1)
        recovery_node = warehouse;
    else
        % 选择原路径位置最接近的 (最小的位置索引)
        [~, closest_idx] = min(backward_candidates);
        recovery_pos = backward_candidates(closest_idx);
        recovery_node = initial_path(recovery_pos + 1);
    end

    % 验证飞行距离约束
    flight_dist = dist_matrix(launch_node, node) + dist_matrix(node, recovery_node);

    if strcmp(drone_type, 'A')
        % A型无人机约束: 需求 ≤ W_A, 航程 ≤ U_A
        if demands(node) <= W_A && flight_dist <= U_A
            drone_A_tasks{end+1, 1} = launch_node;
            drone_A_tasks{end, 2} = node;
            drone_A_tasks{end, 3} = recovery_node;
        else
            % 不满足约束, 将节点退回给车辆
            if ~ismember(node, vehicle_inner)
                % 按原始路径位置插入到车辆路径中
                vehicle_inner = insert_node_by_position(vehicle_inner, node, ...
                    original_pos, remaining_positions, ...
                    initial_path, n_path_nodes);
            end
        end
    else % B型
        % B型无人机约束: 需求 ≤ W_B, 航程 ≤ U_B
        if demands(node) <= W_B && flight_dist <= U_B
            drone_B_tasks{end+1, 1} = launch_node;
            drone_B_tasks{end, 2} = node;
            drone_B_tasks{end, 3} = recovery_node;
        else
            % 不满足约束, 将节点退回给车辆
            if ~ismember(node, vehicle_inner)
                vehicle_inner = insert_node_by_position(vehicle_inner, node, ...
                    original_pos, remaining_positions, ...
                    initial_path, n_path_nodes);
            end
        end
    end
end

% 确保车辆路径不为空
if isempty(vehicle_inner)
    % 如果所有节点都被剥离, 至少保留第一个剥离的节点
    if ~isempty(stripped_nodes)
        fallback_node = stripped_nodes(1).node;
        vehicle_inner = fallback_node;
        % 从无人机任务中移除该节点
        drone_A_tasks = remove_task_by_node(drone_A_tasks, fallback_node);
        drone_B_tasks = remove_task_by_node(drone_B_tasks, fallback_node);
    else
        vehicle_inner = 2;  % 回退
    end
end

vehicle_route = [warehouse, vehicle_inner, warehouse];
end

%%   辅助函数: 按原始路径位置将节点插入到车辆内部节点列表中
function vehicle_inner = insert_node_by_position(vehicle_inner, node, ...
    original_pos, remaining_positions, initial_path, n_path_nodes)
% 根据原始路径中的位置, 将节点插入到车辆内部节点列表的正确位置

% 构建新车辆内部节点列表在原始路径中的参考位置
remaining_pos_sorted = sort(remaining_positions);

% 找到插入位置: 在原始路径中, 该节点应放在哪些保留节点之间
insert_idx = 1;
for i = 1:length(remaining_pos_sorted)
    if remaining_pos_sorted(i) > original_pos
        break;
    end
    insert_idx = i + 1;
end

% 将节点插入到车辆内部节点列表的正确位置
vehicle_inner = [vehicle_inner(1:insert_idx-1), node, vehicle_inner(insert_idx:end)];
end

%% ===========================================================================
%%       全随机无人机任务分配 (初始种群生成专用)
%% ===========================================================================
% 算法思路: 采用与贪婪分配相同的结构化流程, 区别在于:
%   1. 随机选择A/B机型的尝试顺序
%   2. 无轮次上限
%   3. 服务节点拼接方式与贪婪相同（顺序遍历下一个）
%   4. 其余流程 (找最优发射回收点、续航检测、回退) 不变
% ===========================================================================
function [drone_A_tasks, drone_B_tasks, vehicle_route] = ...
    random_drone_assignment(vehicle_route, demands, dist_matrix, ...
    V_A, V_B, W_A, W_B, U_A, U_B)  %#ok<INUSD> % V_A,V_B保留以兼容调用接口

warehouse = vehicle_route(1);

drone_A_tasks = {};
drone_B_tasks = {};

vehicle_inner = vehicle_route(2:end-1);
n_inner = length(vehicle_inner);

is_drone_served = false(1, n_inner);  % 已分配给无人机
is_locked = false(1, n_inner);         % 发射/回收点锁定

% 依次遍历车辆路径上的节点
idx = 1;
while idx <= n_inner
    % 跳过已分配或已锁定的节点
    if is_drone_served(idx) || is_locked(idx)
        idx = idx + 1;
        continue;
    end

    node = vehicle_inner(idx);
    demand_val = demands(node);

    % 判断载重是否满足A或B
    can_A = (demand_val <= W_A);
    can_B = (demand_val <= W_B);

    if ~can_A && ~can_B
        idx = idx + 1;
        continue;  % 都不满足, 由车辆服务
    end

    assigned = false;

    % ----- 随机决定A和B的尝试顺序 -----
    drone_order = [];
    if can_A
        drone_order = [drone_order, 1];  % 1代表A
    end
    if can_B
        drone_order = [drone_order, 2];  % 2代表B
    end
    
    % 随机打乱尝试顺序
    if length(drone_order) > 1
        drone_order = drone_order(randperm(length(drone_order)));
    end

    for d = 1:length(drone_order)
        if drone_order(d) == 1
            % 尝试A型无人机 (顺序遍历服务节点, 与贪婪相同)
            [assigned, drone_A_tasks, is_drone_served, is_locked, new_idx] = ...
                try_assign_drone_task_random('A', node, idx, vehicle_inner, n_inner, ...
                demands, dist_matrix, warehouse, W_A, U_A, ...
                drone_A_tasks, is_drone_served, is_locked);
        else
            % 尝试B型无人机 (顺序遍历服务节点, 与贪婪相同)
            [assigned, drone_B_tasks, is_drone_served, is_locked, new_idx] = ...
                try_assign_drone_task_random('B', node, idx, vehicle_inner, n_inner, ...
                demands, dist_matrix, warehouse, W_B, U_B, ...
                drone_B_tasks, is_drone_served, is_locked);
        end
        if assigned
            idx = new_idx;
            break;
        end
    end

    if ~assigned
        idx = idx + 1;
    end
end

% 去掉无人机服务节点, 剩余节点连接为车辆路径
vehicle_inner = vehicle_inner(~is_drone_served);

if isempty(vehicle_inner)
    % 极端情况: 全部节点被无人机服务, 找回一个给车辆
    original_inner = vehicle_route(2:end-1);
    served_indices = find(is_drone_served);
    if ~isempty(served_indices)
        fallback_node = original_inner(served_indices(1));
        vehicle_inner = fallback_node;
        drone_A_tasks = remove_task_by_node(drone_A_tasks, fallback_node);
        drone_B_tasks = remove_task_by_node(drone_B_tasks, fallback_node);
    else
        vehicle_inner = 2;
    end
end

vehicle_route = [warehouse, vehicle_inner, warehouse];
end

%%贪婪无人机任务分配（多节点拼接版 - 优先A后B）
function [drone_A_tasks, drone_B_tasks, vehicle_route] = ...
    greedy_drone_assignment(vehicle_route, demands, dist_matrix, ...
    W_A, W_B, U_A, U_B)

warehouse = vehicle_route(1);

MAX_A_ROUNDS = 20;   % A型无人机轮次上限
MAX_B_ROUNDS = 15;   % B型无人机轮次上限

drone_A_tasks = {};
drone_B_tasks = {};

vehicle_inner = vehicle_route(2:end-1);
n_inner = length(vehicle_inner);

is_drone_served = false(1, n_inner);  % 已分配给无人机
is_locked = false(1, n_inner);         % 发射/回收点锁定

% 依次遍历车辆路径上的节点
idx = 1;
while idx <= n_inner
    % 跳过已分配或已锁定的节点
    if is_drone_served(idx) || is_locked(idx)
        idx = idx + 1;
        continue;
    end

    node = vehicle_inner(idx);
    demand_val = demands(node);

    % 判断载重是否满足A或B
    can_A = (demand_val <= W_A);
    can_B = (demand_val <= W_B);

    if ~can_A && ~can_B
        idx = idx + 1;
        continue;  % 都不满足, 由车辆服务
    end

    assigned = false;

    % ----- 优先尝试A型无人机 -----
    if can_A && size(drone_A_tasks, 1) < MAX_A_ROUNDS
        [assigned, drone_A_tasks, is_drone_served, is_locked, new_idx] = ...
            try_assign_drone_task('A', node, idx, vehicle_inner, n_inner, ...
            demands, dist_matrix, warehouse, W_A, U_A, ...
            drone_A_tasks, is_drone_served, is_locked);
        if assigned
            idx = new_idx;
            continue;
        end
    end

    % ----- A不行, 尝试B型无人机 -----
    if ~assigned && can_B && size(drone_B_tasks, 1) < MAX_B_ROUNDS
        [assigned, drone_B_tasks, is_drone_served, is_locked, new_idx] = ...
            try_assign_drone_task('B', node, idx, vehicle_inner, n_inner, ...
            demands, dist_matrix, warehouse, W_B, U_B, ...
            drone_B_tasks, is_drone_served, is_locked);
        if assigned
            idx = new_idx;
            continue;
        end
    end

    % 都不行, 标记为车辆服务点
    idx = idx + 1;
end

% 去掉无人机服务节点, 剩余节点连接为车辆路径
vehicle_inner = vehicle_inner(~is_drone_served);

if isempty(vehicle_inner)
    % 极端情况: 全部节点被无人机服务, 找回一个给车辆
    original_inner = vehicle_route(2:end-1);
    served_indices = find(is_drone_served);
    if ~isempty(served_indices)
        fallback_node = original_inner(served_indices(1));
        vehicle_inner = fallback_node;
        drone_A_tasks = remove_task_by_node(drone_A_tasks, fallback_node);
        drone_B_tasks = remove_task_by_node(drone_B_tasks, fallback_node);
    else
        vehicle_inner = 2;
    end
end

vehicle_route = [warehouse, vehicle_inner, warehouse];
end

%% 尝试分配无人机任务 (辅助函数: 先加节点→找发射回收→续航检测→不满足则减节点→再续航检测)
function [assigned, drone_tasks, is_drone_served, is_locked, new_idx] = ...
    try_assign_drone_task(drone_type, start_node, start_idx, vehicle_inner, n_inner, ...
    demands, dist_matrix, warehouse, capacity, max_range, ...
    drone_tasks, is_drone_served, is_locked)

assigned = false;
new_idx = start_idx;

% ----- 步骤1: 拼接服务节点, 只要载重满足就一直加 -----
service_nodes = [start_node];
cumulative_demand = demands(start_node);

next_idx = start_idx + 1;
while next_idx <= n_inner
    if is_drone_served(next_idx) || is_locked(next_idx)
        next_idx = next_idx + 1;
        continue;
    end
    next_node = vehicle_inner(next_idx);
    if cumulative_demand + demands(next_node) <= capacity
        service_nodes = [service_nodes, next_node];
        cumulative_demand = cumulative_demand + demands(next_node);
        next_idx = next_idx + 1;
    else
        break;  % 载重超限, 停止添加
    end
end

% ----- 步骤2: 为多节点服务列表找最优发射/回收点 -----
[best_launch, best_recovery, min_flight_dist] = ...
    find_launch_recovery(service_nodes, vehicle_inner, ...
    is_drone_served, warehouse, dist_matrix);

if best_launch == 0 || best_recovery == 0
    return;
end

% ----- 步骤3: 续航检测, 不满足则去掉添加的节点, 回退到单节点再检测 -----
if min_flight_dist > max_range && length(service_nodes) > 1
    % 去掉添加的后续节点, 只保留第一个服务点
    service_nodes = start_node;
    cumulative_demand = demands(start_node);

    [best_launch, best_recovery, min_flight_dist] = ...
        find_launch_recovery(service_nodes, vehicle_inner, ...
        is_drone_served, warehouse, dist_matrix);

    if best_launch == 0 || best_recovery == 0
        return;
    end
end

% 单节点仍不满足续航 → 放弃
if min_flight_dist > max_range
    return;
end

% ----- 步骤4: 分配成功 -----
if ~isempty(service_nodes)
    drone_tasks{end+1, 1} = best_launch;
    drone_tasks{end, 2} = service_nodes;
    drone_tasks{end, 3} = best_recovery;

    % 标记服务节点 (按节点值查找位置, 而非按连续位置索引)
    last_served_pos = start_idx;
    for s = 1:length(service_nodes)
        pos = find(vehicle_inner == service_nodes(s), 1);
        if ~isempty(pos)
            is_drone_served(pos) = true;
            if pos > last_served_pos
                last_served_pos = pos;
            end
        end
    end

    % 锁定发射/回收点
    launch_pos = find(vehicle_inner == best_launch);
    for lp = launch_pos(:)'
        is_locked(lp) = true;
    end
    recovery_pos = find(vehicle_inner == best_recovery);
    for rp = recovery_pos(:)'
        is_locked(rp) = true;
    end

    assigned = true;
    % 跳到最后一个服务点之后
    new_idx = last_served_pos + 1;
end
end

%% 为服务节点找最优发射/回收点 (候选集: 仓库 + 未服务非自身节点 + 仓库)
function [best_launch, best_recovery, min_flight_dist] = ...
    find_launch_recovery(service_nodes, vehicle_inner, ...
    is_drone_served, warehouse, dist_matrix)

remaining = vehicle_inner(~is_drone_served);
for s = 1:length(service_nodes)
    remaining = remaining(remaining ~= service_nodes(s));
end
candidates = [warehouse, remaining, warehouse];

best_launch = 0;
best_recovery = 0;
min_flight_dist = inf;

n_cand = length(candidates);
for li = 1:n_cand
    for ri = 1:n_cand
        % 发射和回收不能是同一个非仓库节点
        if li == ri && candidates(li) ~= warehouse
            continue;
        end
        launch_cand = candidates(li);
        recovery_cand = candidates(ri);

        % 飞行距离: 发射→服务点1→...→服务点n→回收
        flight_dist = dist_matrix(launch_cand, service_nodes(1));
        for s = 1:length(service_nodes)-1
            flight_dist = flight_dist + dist_matrix(service_nodes(s), service_nodes(s+1));
        end
        flight_dist = flight_dist + dist_matrix(service_nodes(end), recovery_cand);

        if flight_dist < min_flight_dist
            min_flight_dist = flight_dist;
            best_launch = launch_cand;
            best_recovery = recovery_cand;
        end
    end
end
end

%% 尝试分配无人机任务 (随机版, 与贪婪版唯一区别: 尝试的顺序遍历拼接, 无轮次上限)
function [assigned, drone_tasks, is_drone_served, is_locked, new_idx] = ...
    try_assign_drone_task_random(drone_type, start_node, start_idx, vehicle_inner, n_inner, ...
    demands, dist_matrix, warehouse, capacity, max_range, ...
    drone_tasks, is_drone_served, is_locked)

assigned = false;
new_idx = start_idx;

% ----- 步骤1: 顺序拼接服务节点, 只要载重满足就一直加 (与贪婪相同) -----
service_nodes = [start_node];
cumulative_demand = demands(start_node);

next_idx = start_idx + 1;
while next_idx <= n_inner
    if is_drone_served(next_idx) || is_locked(next_idx)
        next_idx = next_idx + 1;
        continue;
    end
    next_node = vehicle_inner(next_idx);
    if cumulative_demand + demands(next_node) <= capacity
        service_nodes = [service_nodes, next_node];
        cumulative_demand = cumulative_demand + demands(next_node);
        next_idx = next_idx + 1;
    else
        break;  % 载重超限, 停止添加
    end
end

% ----- 步骤2: 为多节点服务列表找最优发射/回收点 -----
[best_launch, best_recovery, min_flight_dist] = ...
    find_launch_recovery(service_nodes, vehicle_inner, ...
    is_drone_served, warehouse, dist_matrix);

if best_launch == 0 || best_recovery == 0
    return;
end

% ----- 步骤3: 续航检测, 不满足则去掉添加的节点, 回退到单节点再检测 -----
if min_flight_dist > max_range && length(service_nodes) > 1
    service_nodes = start_node;
    cumulative_demand = demands(start_node);

    [best_launch, best_recovery, min_flight_dist] = ...
        find_launch_recovery(service_nodes, vehicle_inner, ...
        is_drone_served, warehouse, dist_matrix);

    if best_launch == 0 || best_recovery == 0
        return;
    end
end

% 单节点仍不满足续航 → 放弃
if min_flight_dist > max_range
    return;
end

% ----- 步骤4: 分配成功 -----
if ~isempty(service_nodes)
    drone_tasks{end+1, 1} = best_launch;
    drone_tasks{end, 2} = service_nodes;
    drone_tasks{end, 3} = best_recovery;

    last_served_pos = start_idx;
    for s = 1:length(service_nodes)
        pos = find(vehicle_inner == service_nodes(s), 1);
        if ~isempty(pos)
            is_drone_served(pos) = true;
            if pos > last_served_pos
                last_served_pos = pos;
            end
        end
    end

    % 锁定发射/回收点
    launch_pos = find(vehicle_inner == best_launch);
    for lp = launch_pos(:)'
        is_locked(lp) = true;
    end
    recovery_pos = find(vehicle_inner == best_recovery);
    for rp = recovery_pos(:)'
        is_locked(rp) = true;
    end

    assigned = true;
    new_idx = last_served_pos + 1;
end
end

%%                 染色体校验与修复
function chromosome = validate_and_repair(chromosome, n_nodes, warehouse, ...
    demands, dist_matrix, V_A, V_B, W_A, W_B, U_A, U_B)

vr = chromosome.vehicle_route;

% 1. 闭合性校验
if vr(1) ~= warehouse
    vr = [warehouse, vr];
end
if vr(end) ~= warehouse
    vr = [vr, warehouse];
end

% 2. 去重
vr_inner = vr(2:end-1);
vr_inner_unique = unique(vr_inner, 'stable');
vr = [warehouse, vr_inner_unique, warehouse];

% 3. 确保所有节点都被服务
all_service_nodes = vr(2:end-1);
drone_A_nodes = [];
drone_B_nodes = [];

for k = 1:size(chromosome.drone_A_tasks, 1)
    s = chromosome.drone_A_tasks{k,2};
    if iscell(s)
        drone_A_nodes = [drone_A_nodes, cell2mat(s)];
    else
        drone_A_nodes = [drone_A_nodes, s];
    end
end
for k = 1:size(chromosome.drone_B_tasks, 1)
    s = chromosome.drone_B_tasks{k,2};
    if iscell(s)
        drone_B_nodes = [drone_B_nodes, cell2mat(s)];
    else
        drone_B_nodes = [drone_B_nodes, s];
    end
end

drone_served = [drone_A_nodes(:)', drone_B_nodes(:)'];

% 检查是否有节点重复服务
vehicle_only = setdiff(all_service_nodes, drone_served);
all_covered = unique([vehicle_only(:)', drone_A_nodes(:)', drone_B_nodes(:)']);

missing_nodes = setdiff(2:n_nodes, all_covered);
if ~isempty(missing_nodes)
    vr = [warehouse, vehicle_only, missing_nodes, warehouse];
end

% 4. 无人机任务约束校验
drone_A_tasks_new = {};
orphan_nodes_A = [];  % 记录被修复丢弃的节点
for k = 1:size(chromosome.drone_A_tasks, 1)
    task = chromosome.drone_A_tasks(k, :);
    launch = task{1};
    service = task{2};
    recovery = task{3};
    if iscell(service)
        service = cell2mat(service);
    end
    if isempty(service)
        continue;
    end

    flight_dist = dist_matrix(launch, service(1)) + dist_matrix(service(end), recovery);
    for s = 1:length(service)-1
        flight_dist = flight_dist + dist_matrix(service(s), service(s+1));
    end

    total_demand = sum(demands(service));

    if flight_dist <= U_A && total_demand <= W_A
        drone_A_tasks_new{end+1, 1} = launch;
        drone_A_tasks_new{end, 2} = service;
        drone_A_tasks_new{end, 3} = recovery;
    else
        % 贪心修复: 拆分/缩减任务, 记录被丢弃的节点
        original_nodes = service;
        repaired = greedy_task_repair(launch, service, recovery, demands, ...
            dist_matrix, 'A', W_A, U_A);
        kept_nodes = [];
        for r = 1:size(repaired, 1)
            drone_A_tasks_new{end+1, 1} = repaired{r,1};
            drone_A_tasks_new{end, 2} = repaired{r,2};
            drone_A_tasks_new{end, 3} = repaired{r,3};
            kept_nodes = [kept_nodes, repaired{r,2}];
        end
        orphan_nodes_A = [orphan_nodes_A, setdiff(original_nodes, kept_nodes)];
    end
end
chromosome.drone_A_tasks = drone_A_tasks_new;

drone_B_tasks_new = {};
orphan_nodes_B = [];  % 记录被修复丢弃的节点
for k = 1:size(chromosome.drone_B_tasks, 1)
    task = chromosome.drone_B_tasks(k, :);
    launch = task{1};
    service = task{2};
    recovery = task{3};
    if iscell(service)
        service = cell2mat(service);
    end
    if isempty(service)
        continue;
    end

    flight_dist = dist_matrix(launch, service(1)) + dist_matrix(service(end), recovery);
    for s = 1:length(service)-1
        flight_dist = flight_dist + dist_matrix(service(s), service(s+1));
    end

    total_demand = sum(demands(service));

    if flight_dist <= U_B && total_demand <= W_B
        drone_B_tasks_new{end+1, 1} = launch;
        drone_B_tasks_new{end, 2} = service;
        drone_B_tasks_new{end, 3} = recovery;
    else
        % 贪心修复: 拆分/缩减任务, 记录被丢弃的节点
        original_nodes = service;
        repaired = greedy_task_repair(launch, service, recovery, demands, ...
            dist_matrix, 'B', W_B, U_B);
        kept_nodes = [];
        for r = 1:size(repaired, 1)
            drone_B_tasks_new{end+1, 1} = repaired{r,1};
            drone_B_tasks_new{end, 2} = repaired{r,2};
            drone_B_tasks_new{end, 3} = repaired{r,3};
            kept_nodes = [kept_nodes, repaired{r,2}];
        end
        orphan_nodes_B = [orphan_nodes_B, setdiff(original_nodes, kept_nodes)];
    end
end
chromosome.drone_B_tasks = drone_B_tasks_new;

% 将被丢弃的节点加回车辆路径
orphan_all = [orphan_nodes_A, orphan_nodes_B];
if ~isempty(orphan_all)
    vr_inner = vr(2:end-1);
    vr_inner = [vr_inner, orphan_all];
end

% 剔除发射/回收点不在车辆路径中的无人机任务, 将其服务节点回收至车辆
vr_nodes_current = unique(vr_inner);
reclaimed_nodes = [];
drone_A_valid = {};
for k = 1:size(chromosome.drone_A_tasks, 1)
    launch = chromosome.drone_A_tasks{k,1};
    recovery = chromosome.drone_A_tasks{k,3};
    if ismember(launch, vr_nodes_current) && ismember(recovery, vr_nodes_current)
        drone_A_valid{end+1,1} = launch;
        drone_A_valid{end,2} = chromosome.drone_A_tasks{k,2};
        drone_A_valid{end,3} = recovery;
    else
        s = chromosome.drone_A_tasks{k,2};
        if iscell(s), s = cell2mat(s); end
        reclaimed_nodes = [reclaimed_nodes, s];
    end
end
chromosome.drone_A_tasks = drone_A_valid;

drone_B_valid = {};
for k = 1:size(chromosome.drone_B_tasks, 1)
    launch = chromosome.drone_B_tasks{k,1};
    recovery = chromosome.drone_B_tasks{k,3};
    if ismember(launch, vr_nodes_current) && ismember(recovery, vr_nodes_current)
        drone_B_valid{end+1,1} = launch;
        drone_B_valid{end,2} = chromosome.drone_B_tasks{k,2};
        drone_B_valid{end,3} = recovery;
    else
        s = chromosome.drone_B_tasks{k,2};
        if iscell(s), s = cell2mat(s); end
        reclaimed_nodes = [reclaimed_nodes, s];
    end
end
chromosome.drone_B_tasks = drone_B_valid;

if ~isempty(reclaimed_nodes)
    vr_inner = [vr_inner, reclaimed_nodes];
end

vr = [warehouse, unique(vr_inner, 'stable'), warehouse];

chromosome.vehicle_route = vr;
end

%% ===========================================================================
%%                 贪心任务修复
%% ===========================================================================
function repaired_tasks = greedy_task_repair(launch, service_nodes, recovery, ...
    demands, dist_matrix, drone_type, W_limit, U_limit)

repaired_tasks = {};

if isempty(service_nodes)
    return;
end

% 如果单个节点超载, 移除最远节点
current_demand = sum(demands(service_nodes));
current_nodes = service_nodes;

while current_demand > W_limit && length(current_nodes) > 1
    % 移除距离发射点最远的节点
    dists_to_launch = dist_matrix(launch, current_nodes);
    [~, idx_remove] = max(dists_to_launch);
    current_nodes(idx_remove) = [];
    current_demand = sum(demands(current_nodes));
end

if current_demand <= W_limit && ~isempty(current_nodes)
    % 计算航程
    flight_dist = dist_matrix(launch, current_nodes(1)) + dist_matrix(current_nodes(end), recovery);
    for s = 1:length(current_nodes)-1
        flight_dist = flight_dist + dist_matrix(current_nodes(s), current_nodes(s+1));
    end

    if flight_dist <= U_limit
        repaired_tasks{1,1} = launch;
        repaired_tasks{1,2} = current_nodes;
        repaired_tasks{1,3} = recovery;
    else
        % 超航程, 拆分 - 同时确保每份都不超载
        n = length(current_nodes);
        mid = ceil(n/2);

        first_half = current_nodes(1:mid);
        second_half = current_nodes(mid+1:end);

        % 第一份: 如果不超载则接受
        if sum(demands(first_half)) <= W_limit
            repaired_tasks{1,1} = launch;
            repaired_tasks{1,2} = first_half;
            repaired_tasks{1,3} = recovery;
        else
            % 第一份仍超载, 递归缩减
            for ni = 1:length(first_half)
                if demands(first_half(ni)) <= W_limit
                    repaired_tasks{end+1,1} = launch;
                    repaired_tasks{end,2} = first_half(ni);
                    repaired_tasks{end,3} = recovery;
                end
            end
        end

        % 第二份: 同样验证
        if ~isempty(second_half)
            if sum(demands(second_half)) <= W_limit
                repaired_tasks{end+1,1} = launch;
                repaired_tasks{end,2} = second_half;
                repaired_tasks{end,3} = recovery;
            else
                % 第二份超载, 递归缩减
                for ni = 1:length(second_half)
                    if demands(second_half(ni)) <= W_limit
                        repaired_tasks{end+1,1} = launch;
                        repaired_tasks{end,2} = second_half(ni);
                        repaired_tasks{end,3} = recovery;
                    end
                end
            end
        end
    end
end
end

%%适应度评估 (使用预计算的距离矩阵)
function fitness = evaluate_fitness(chromosome, dist_matrix, demands, n_nodes, ...
    V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
    service_D_A, service_D_B, At_max, M)

[~, ~, ~, total_time, ~] = decode_solution(chromosome, dist_matrix, demands, n_nodes, ...
    V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
    service_D_A, service_D_B, At_max, M);

% 适应度 = 1/总时间 (公式5.1)
if total_time > 0
    fitness = 1 / total_time;
else
    fitness = 0;
end

% % 惩罚不可行解
% penalty = 0;

% % 检查载重约束
% for k = 1:size(chromosome.drone_A_tasks, 1)
%     service = chromosome.drone_A_tasks{k,2};
%     if iscell(service), service = cell2mat(service); end
%     if sum(demands(service)) > W_A
%         penalty = penalty + 100;
%     end
% end
% for k = 1:size(chromosome.drone_B_tasks, 1)
%     service = chromosome.drone_B_tasks{k,2};
%     if iscell(service), service = cell2mat(service); end
%     if sum(demands(service)) > W_B
%         penalty = penalty + 100;
%     end
% end

% % 检查航程约束
% for k = 1:size(chromosome.drone_A_tasks, 1)
%     service = chromosome.drone_A_tasks{k,2};
%     if iscell(service), service = cell2mat(service); end
%     launch = chromosome.drone_A_tasks{k,1};
%     recovery = chromosome.drone_A_tasks{k,3};
%     flight_dist = dist_matrix(launch, service(1)) + dist_matrix(service(end), recovery);
%     for s = 1:length(service)-1
%         flight_dist = flight_dist + dist_matrix(service(s), service(s+1));
%     end
%     if flight_dist > U_A
%         penalty = penalty + 100;
%     end
% end
% for k = 1:size(chromosome.drone_B_tasks, 1)
%     service = chromosome.drone_B_tasks{k,2};
%     if iscell(service), service = cell2mat(service); end
%     launch = chromosome.drone_B_tasks{k,1};
%     recovery = chromosome.drone_B_tasks{k,3};
%     flight_dist = dist_matrix(launch, service(1)) + dist_matrix(service(end), recovery);
%     for s = 1:length(service)-1
%         flight_dist = flight_dist + dist_matrix(service(s), service(s+1));
%     end
%     if flight_dist > U_B
%         penalty = penalty + 100;
%     end
% end

% if penalty > 0
%     fitness = fitness / (1 + penalty / 100);
% end
end

%%解码求解 (计算总配送时间 - 迭代修正无人机发射时间)
function [vehicle_route, drone_tasks_A, drone_tasks_B, total_time, details] = ...
    decode_solution(chromosome, dist_matrix, demands, n_nodes, ...
    V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
    service_D_A, service_D_B, At_max, M)

% ===== 步骤1: 提取路径和任务 =====
vehicle_route = chromosome.vehicle_route;
drone_tasks_A = chromosome.drone_A_tasks;
drone_tasks_B = chromosome.drone_B_tasks;
warehouse = vehicle_route(1);

% ===== 步骤2: 构建无人机任务结构体数组 =====
all_drone_info = [];

% 处理无人机A任务
for k = 1:size(drone_tasks_A, 1)
    L = drone_tasks_A{k, 1};
    SN = drone_tasks_A{k, 2};
    if iscell(SN), SN = cell2mat(SN); end
    R = drone_tasks_A{k, 3};
    all_drone_info = [all_drone_info, struct('launch', L, 'service_nodes', SN, ...
        'recovery', R, 'type', 'A', 'task_idx', k, 't_drone_back', 0)];
end

% 处理无人机B任务
for k = 1:size(drone_tasks_B, 1)
    L = drone_tasks_B{k, 1};
    SN = drone_tasks_B{k, 2};
    if iscell(SN), SN = cell2mat(SN); end
    R = drone_tasks_B{k, 3};
    all_drone_info = [all_drone_info, struct('launch', L, 'service_nodes', SN, ...
        'recovery', R, 'type', 'B', 'task_idx', k, 't_drone_back', 0)];
end

% ===== 步骤3: 迭代计算真实时间 (无人机发射时间依赖车辆到达, 车辆到达又依赖无人机回收等待) =====
real_arrival = zeros(1, n_nodes) - 1;
real_depart  = zeros(1, n_nodes) - 1;

max_iter = 20;
for iter = 1:max_iter
    % --- 3a. 根据当前 real_arrival 更新所有无人机的飞回时间 ---
    drone_updated = false;
    for d = 1:length(all_drone_info)
        L = all_drone_info(d).launch;
        SN = all_drone_info(d).service_nodes;
        R = all_drone_info(d).recovery;
        drone_type = all_drone_info(d).type;
        
        % 无人机起飞时间 = 车辆真实到达发射点的时间
        if real_arrival(L) >= 0
            t_launch = real_arrival(L);
        else
            t_launch = 0;
        end
        
        if strcmp(drone_type, 'A')
            V_drone = V_A;
            service_D = service_D_A;
        else
            V_drone = V_B;
            service_D = service_D_B;
        end
        
        t_drone = t_launch;
        t_drone = t_drone + dist_matrix(L, SN(1)) / V_drone;
        t_drone = t_drone + service_D;
        for s = 1:length(SN)-1
            t_drone = t_drone + dist_matrix(SN(s), SN(s+1)) / V_drone;
            t_drone = t_drone + service_D;
        end
        t_drone = t_drone + dist_matrix(SN(end), R) / V_drone;
        
        if abs(t_drone - all_drone_info(d).t_drone_back) > 1e-6
            all_drone_info(d).t_drone_back = t_drone;
            drone_updated = true;
        end
    end
    
    % 首次迭代强制更新
    if iter == 1
        drone_updated = true;
    end
    
    if ~drone_updated && iter > 1
        break;  % 收敛, 退出迭代
    end
    
    % --- 3b. 构建回收点时间映射 ---
    recovery_map = cell(1, n_nodes);
    for d = 1:length(all_drone_info)
        r_node = all_drone_info(d).recovery;
        if r_node > 0 && r_node <= n_nodes
            recovery_map{r_node} = [recovery_map{r_node}, all_drone_info(d).t_drone_back];
        end
    end
    
    % --- 3c. 重新计算车辆真实到达/离开时间 (考虑无人机回收等待) ---
    real_arrival(:) = -1;
    real_depart(:) = -1;
    real_arrival(warehouse) = 0;
    real_depart(warehouse) = 0;
    
    current_time = 0;
    for i = 1:length(vehicle_route)-1
        from_node = vehicle_route(i);
        to_node   = vehicle_route(i+1);
        
        travel_time = dist_matrix(from_node, to_node) / V_T;
        current_time = current_time + travel_time;
        
        if real_arrival(to_node) < 0
            real_arrival(to_node) = current_time;
        else
            real_arrival(to_node) = max(real_arrival(to_node), current_time);
        end
        
        % 检查是否有无人机需要在该点回收
        drone_back_times = recovery_map{to_node};
        if ~isempty(drone_back_times)
            latest_back = max(drone_back_times);
            if current_time < latest_back
                current_time = latest_back;  % 等待无人机
            end
        end
        
        % 仓库节点不需要服务时间
        if to_node ~= warehouse
            current_time = current_time + service_T;
        end
        real_depart(to_node) = current_time;
    end
end

% ===== 步骤4: 计算总时间 =====
% 车辆到达仓库时间 (含仓库处无人机回收等待, 不含仓库服务时间)
vehicle_arrival_at_warehouse = real_arrival(warehouse);

% 计算无人机任务完成时间
all_drone_end_times = [];
for d = 1:length(all_drone_info)
    r_node = all_drone_info(d).recovery;
    if r_node > 0 && r_node <= n_nodes && real_arrival(r_node) >= 0
        drone_end = max(all_drone_info(d).t_drone_back, real_arrival(r_node));
    else
        drone_end = all_drone_info(d).t_drone_back;
    end
    all_drone_end_times = [all_drone_end_times, drone_end];
end

% 总时间 = 所有任务完成的最晚时间
total_time = vehicle_arrival_at_warehouse;
if ~isempty(all_drone_end_times)
    total_time = max(total_time, max(all_drone_end_times));
end

% 保存详细信息
details.vehicle_return_time = vehicle_arrival_at_warehouse;
details.drone_end_times = all_drone_end_times;
details.real_arrival = real_arrival;
details.real_depart = real_depart;
end

%% ===========================================================================
%%                 解验证
%% ===========================================================================
function ok = verify_solution(vehicle_route, drone_tasks_A, drone_tasks_B, ...
    coords, demands, dist_matrix, V_A, V_B, W_A, W_B, U_A, U_B)

ok = true;
n_nodes = size(coords, 1);

% 1. 闭合性
if vehicle_route(1) ~= 1 || vehicle_route(end) ~= 1
    fprintf('  错误: 车辆路径不闭合\n');
    ok = false;
end

% 2. 所有节点是否都被服务 (节点1是仓库,不需要被服务)
served_nodes = unique(vehicle_route(2:end-1));
for k = 1:size(drone_tasks_A, 1)
    s = drone_tasks_A{k,2};
    if iscell(s), s = cell2mat(s); end
    served_nodes = [served_nodes, s];
end
for k = 1:size(drone_tasks_B, 1)
    s = drone_tasks_B{k,2};
    if iscell(s), s = cell2mat(s); end
    served_nodes = [served_nodes, s];
end

served_nodes = unique(served_nodes);

% 2b. 检查是否有节点被重复服务 (既在车辆路径又在无人机任务中)
vehicle_inner = vehicle_route(2:end-1);
drone_served = [];
for k = 1:size(drone_tasks_A, 1)
    s = drone_tasks_A{k,2};
    if iscell(s), s = cell2mat(s); end
    drone_served = [drone_served, s];
end
for k = 1:size(drone_tasks_B, 1)
    s = drone_tasks_B{k,2};
    if iscell(s), s = cell2mat(s); end
    drone_served = [drone_served, s];
end
duplicate_in_vehicle = intersect(vehicle_inner, drone_served);
if ~isempty(duplicate_in_vehicle)
    fprintf('  错误: 节点 %s 同时被车辆和无人机服务\n', num2str(duplicate_in_vehicle));
    ok = false;
end

all_nodes = 2:n_nodes;
missing = setdiff(all_nodes, served_nodes);
if ~isempty(missing)
    fprintf('  错误: 节点 %s 未被服务\n', num2str(missing));
    ok = false;
end

% 3. 载重约束
for k = 1:size(drone_tasks_A, 1)
    s = drone_tasks_A{k,2};
    if iscell(s), s = cell2mat(s); end
    if sum(demands(s)) > W_A
        fprintf('  错误: 无人机A任务%d超载\n', k);
        ok = false;
    end
end
for k = 1:size(drone_tasks_B, 1)
    s = drone_tasks_B{k,2};
    if iscell(s), s = cell2mat(s); end
    if sum(demands(s)) > W_B
        fprintf('  错误: 无人机B任务%d超载\n', k);
        ok = false;
    end
end

% 4. 航程约束
for k = 1:size(drone_tasks_A, 1)
    s = drone_tasks_A{k,2};
    if iscell(s), s = cell2mat(s); end
    launch = drone_tasks_A{k,1};
    recovery = drone_tasks_A{k,3};
    flight_dist = dist_matrix(launch, s(1)) + dist_matrix(s(end), recovery);
    for i = 1:length(s)-1
        flight_dist = flight_dist + dist_matrix(s(i), s(i+1));
    end
    if flight_dist > U_A
        fprintf('  错误: 无人机A任务%d超航程 (%.1f > %.1f)\n', k, flight_dist, U_A);
        ok = false;
    end
end
for k = 1:size(drone_tasks_B, 1)
    s = drone_tasks_B{k,2};
    if iscell(s), s = cell2mat(s); end
    launch = drone_tasks_B{k,1};
    recovery = drone_tasks_B{k,3};
    flight_dist = dist_matrix(launch, s(1)) + dist_matrix(s(end), recovery);
    for i = 1:length(s)-1
        flight_dist = flight_dist + dist_matrix(s(i), s(i+1));
    end
    if flight_dist > U_B
        fprintf('  错误: 无人机B任务%d超航程 (%.1f > %.1f)\n', k, flight_dist, U_B);
        ok = false;
    end
end

% 5. 发射/回收点必须是车辆服务点
vehicle_nodes = unique(vehicle_route);
for k = 1:size(drone_tasks_A, 1)
    if ~ismember(drone_tasks_A{k,1}, vehicle_nodes)
        fprintf('  错误: 无人机A任务%d发射点%d不是车辆服务点\n', k, drone_tasks_A{k,1});
        ok = false;
    end
    if ~ismember(drone_tasks_A{k,3}, vehicle_nodes)
        fprintf('  错误: 无人机A任务%d回收点%d不是车辆服务点\n', k, drone_tasks_A{k,3});
        ok = false;
    end
end
for k = 1:size(drone_tasks_B, 1)
    if ~ismember(drone_tasks_B{k,1}, vehicle_nodes)
        fprintf('  错误: 无人机B任务%d发射点%d不是车辆服务点\n', k, drone_tasks_B{k,1});
        ok = false;
    end
    if ~ismember(drone_tasks_B{k,3}, vehicle_nodes)
        fprintf('  错误: 无人机B任务%d回收点%d不是车辆服务点\n', k, drone_tasks_B{k,3});
        ok = false;
    end
end

if ok
    fprintf('  所有约束验证通过\n');
end
end

%%锦标赛选择
function selected = tournament_select(population, fitness, k)
pop_size = length(population);
% 兼容旧版MATLAB的randperm
n_select = min(k, pop_size);
idx = randperm(pop_size);
idx = idx(1:n_select);
[~, best] = max(fitness(idx));
selected = population{idx(best)};
end

%%动态协同交叉策略
function offspring = dynamic_cooperative_crossover(parent1, parent2, ...
    demands, dist_matrix, V_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
    n_nodes, warehouse)

% 1. 车辆路径交叉 - 改进顺序交叉 (MOX)
vr1 = parent1.vehicle_route;
vr2 = parent2.vehicle_route;

% 随机选择交叉片段
inner_len = length(vr1) - 2;  % 去掉首尾仓库
if inner_len >= 2
    cp1 = randi(inner_len - 1);
    cp2 = randi([cp1+1, inner_len]);

    % 提取父代1的交叉片段
    segment = vr1(cp1+1:cp2+1);  % +1因为从第2个位置开始

    % 从父代2填充剩余节点
    remaining = [];
    for i = 2:length(vr2)-1  % 跳过首尾仓库
        if ~ismember(vr2(i), segment)
            remaining = [remaining, vr2(i)];
        end
    end

    % 构建子代路径
    if cp1 <= length(remaining)
        child_vr = [warehouse, remaining(1:cp1), segment, remaining(cp1+1:end), warehouse];
    else
        child_vr = [warehouse, remaining, segment, warehouse];
    end

    % 闭合性修复
    if child_vr(1) ~= warehouse
        child_vr = [warehouse, child_vr];
    end
    if child_vr(end) ~= warehouse
        child_vr = [child_vr, warehouse];
    end

    % 去重
    child_vr_unique = [warehouse];
    for i = 2:length(child_vr)-1
        if ~ismember(child_vr(i), child_vr_unique)
            child_vr_unique = [child_vr_unique, child_vr(i)];
        end
    end
    child_vr = [child_vr_unique, warehouse];

    % 确保所有节点都在路径中
    all_inner = 2:n_nodes;
    missing_vr = setdiff(all_inner, child_vr(2:end-1));
    if ~isempty(missing_vr)
        child_vr = [warehouse, child_vr(2:end-1), missing_vr, warehouse];
    end
else
    child_vr = vr1;
end

% 2. 无人机任务交叉 - 类型匹配交叉
child_drone_A = {};
child_drone_B = {};

% 处理A型任务
p1_A = parent1.drone_A_tasks;
p2_A = parent2.drone_A_tasks;
child_drone_A = type_match_crossover(p1_A, p2_A, demands, dist_matrix, ...
    W_A, U_A, n_nodes);

% 处理B型任务
p1_B = parent1.drone_B_tasks;
p2_B = parent2.drone_B_tasks;
child_drone_B = type_match_crossover(p1_B, p2_B, demands, dist_matrix, ...
    W_B, U_B, n_nodes);

% 3. 协同优化 - 时间窗口校验
offspring.vehicle_route = child_vr;
offspring.drone_A_tasks = child_drone_A;
offspring.drone_B_tasks = child_drone_B;

% 修复冲突
offspring = resolve_drone_conflicts(offspring, n_nodes, warehouse);
end

%%类型匹配交叉
function child_tasks = type_match_crossover(p1_tasks, p2_tasks, demands, ...
    dist_matrix, W_limit, U_limit, n_nodes)

n1 = size(p1_tasks, 1);
n2 = size(p2_tasks, 1);

if n1 == 0 && n2 == 0
    child_tasks = {};
    return;
elseif n1 == 0
    child_tasks = p2_tasks;
    return;
elseif n2 == 0
    child_tasks = p1_tasks;
    return;
end

% 在同类型任务中随机选择交叉位置
if rand() < 0.5
    % 交换服务节点序列
    child_tasks = p1_tasks;
    swap_idx1 = randi(n1);
    swap_idx2 = randi(n2);

    temp_service = child_tasks{swap_idx1, 2};
    child_tasks{swap_idx1, 2} = p2_tasks{swap_idx2, 2};

    % 约束校验
    service = child_tasks{swap_idx1, 2};
    if iscell(service), service = cell2mat(service); end
    launch = child_tasks{swap_idx1, 1};
    recovery = child_tasks{swap_idx1, 3};

    flight_dist = dist_matrix(launch, service(1)) + dist_matrix(service(end), recovery);
    for s = 1:length(service)-1
        flight_dist = flight_dist + dist_matrix(service(s), service(s+1));
    end
    total_demand = sum(demands(service));

    if total_demand > W_limit || flight_dist > U_limit
        % 贪心修复
        child_tasks{swap_idx1, 2} = temp_service;  % 恢复
    end
else
    % 随机选择父代
    if rand() < 0.5
        child_tasks = p1_tasks;
    else
        child_tasks = p2_tasks;
    end
end
end

%%辅助函数: 从无人机任务中移除指定节点
function tasks = remove_task_by_node(tasks, node_to_remove)
if isempty(tasks)
    return;
end
new_tasks = {};
for k = 1:size(tasks, 1)
    s = tasks{k, 2};
    if iscell(s), s = cell2mat(s); end
    if ~isequal(s, node_to_remove) && ~ismember(node_to_remove, s)
        new_tasks{end+1, 1} = tasks{k, 1};
        new_tasks{end, 2} = tasks{k, 2};
        new_tasks{end, 3} = tasks{k, 3};
    end
end
tasks = new_tasks;
end

%%解决无人机任务冲突
function offspring = resolve_drone_conflicts(offspring, n_nodes, warehouse)
% 收集所有被无人机服务的节点
drone_A_nodes = [];
drone_B_nodes = [];

for k = 1:size(offspring.drone_A_tasks, 1)
    s = offspring.drone_A_tasks{k,2};
    if iscell(s), s = cell2mat(s); end
    drone_A_nodes = [drone_A_nodes, s];
end
for k = 1:size(offspring.drone_B_tasks, 1)
    s = offspring.drone_B_tasks{k,2};
    if iscell(s), s = cell2mat(s); end
    drone_B_nodes = [drone_B_nodes, s];
end

% 检查A和B之间是否有冲突
conflict_nodes = intersect(drone_A_nodes, drone_B_nodes);
if ~isempty(conflict_nodes)
    % 从B中移除冲突节点
    drone_B_tasks_new = {};
    for k = 1:size(offspring.drone_B_tasks, 1)
        s = offspring.drone_B_tasks{k,2};
        if iscell(s), s = cell2mat(s); end
        s_new = setdiff(s, conflict_nodes);
        if ~isempty(s_new)
            drone_B_tasks_new{end+1, 1} = offspring.drone_B_tasks{k,1};
            drone_B_tasks_new{end, 2} = s_new;
            drone_B_tasks_new{end, 3} = offspring.drone_B_tasks{k,3};
        end
    end
    offspring.drone_B_tasks = drone_B_tasks_new;
end

% 剔除发射/回收点不在车辆路径中的无人机任务 (将服务节点回收)
vr_nodes = offspring.vehicle_route(2:end-1);

drone_A_tasks_valid = {};
drone_A_nodes = [];
for k = 1:size(offspring.drone_A_tasks, 1)
    launch = offspring.drone_A_tasks{k,1};
    recovery = offspring.drone_A_tasks{k,3};
    if ismember(launch, vr_nodes) && ismember(recovery, vr_nodes)
        drone_A_tasks_valid{end+1,1} = launch;
        drone_A_tasks_valid{end,2} = offspring.drone_A_tasks{k,2};
        drone_A_tasks_valid{end,3} = recovery;
        s = offspring.drone_A_tasks{k,2};
        if iscell(s), s = cell2mat(s); end
        drone_A_nodes = [drone_A_nodes, s];
    end
end
offspring.drone_A_tasks = drone_A_tasks_valid;

drone_B_tasks_valid = {};
drone_B_nodes = [];
for k = 1:size(offspring.drone_B_tasks, 1)
    launch = offspring.drone_B_tasks{k,1};
    recovery = offspring.drone_B_tasks{k,3};
    if ismember(launch, vr_nodes) && ismember(recovery, vr_nodes)
        drone_B_tasks_valid{end+1,1} = launch;
        drone_B_tasks_valid{end,2} = offspring.drone_B_tasks{k,2};
        drone_B_tasks_valid{end,3} = recovery;
        s = offspring.drone_B_tasks{k,2};
        if iscell(s), s = cell2mat(s); end
        drone_B_nodes = [drone_B_nodes, s];
    end
end
offspring.drone_B_tasks = drone_B_tasks_valid;

% 确保车辆路径不包含无人机服务节点
drone_all_nodes = unique([drone_A_nodes, drone_B_nodes]);
vr_inner = offspring.vehicle_route(2:end-1);
vr_inner_new = setdiff(vr_inner, drone_all_nodes, 'stable');
% 恢复孤儿节点: 那些既不在车辆也不在无人机的节点
drone_served_all = unique([drone_A_nodes, drone_B_nodes]);
all_nodes = 2:n_nodes;
orphan_nodes = setdiff(all_nodes, [vr_inner_new, drone_served_all]);
if ~isempty(orphan_nodes)
    vr_inner_new = [vr_inner_new, orphan_nodes];
end

if isempty(vr_inner_new)
    % 至少保留一个节点给车辆
    if ~isempty(drone_all_nodes)
        vr_inner_new = drone_all_nodes(1);
    else
        vr_inner_new = 2;  % 回退
    end
end
offspring.vehicle_route = [warehouse, vr_inner_new, warehouse];
end

%%约束感知变异策略
function offspring = constraint_aware_mutation(chromosome, demands, dist_matrix, ...
    V_A, V_B, W_A, W_B, U_A, U_B, n_nodes, warehouse)

offspring = chromosome;

% % 1. 车辆路径变异 - 2-opt路径翻转
% vr = offspring.vehicle_route;
% if length(vr) >= 5  % 至少需要仓库 + 2个中间节点 + 仓库
%     % 随机选两个位置 (在内部节点中)
%     inner = vr(2:end-1);
%     if length(inner) >= 2
%         i = randi(length(inner)-1);
%         j = randi([i+1, length(inner)]);

%         % 翻转i+1到j段
%         inner(i:j) = fliplr(inner(i:j));
%         vr = [warehouse, inner, warehouse];

%         % 检查闭合性
%         if vr(1) == warehouse && vr(end) == warehouse
%             offspring.vehicle_route = vr;
%         end
%     end
% end

% 2. 无人机任务变异

% 2a. 机型切换变异 (概率0.3)
if rand() < 0.3
    if ~isempty(offspring.drone_A_tasks) && rand() < 0.5
        % 尝试将A任务切换为B
        k = randi(size(offspring.drone_A_tasks, 1));
        service = offspring.drone_A_tasks{k,2};
        if iscell(service), service = cell2mat(service); end
        launch = offspring.drone_A_tasks{k,1};
        recovery = offspring.drone_A_tasks{k,3};

        flight_dist = dist_matrix(launch, service(1)) + dist_matrix(service(end), recovery);
        for s = 1:length(service)-1
            flight_dist = flight_dist + dist_matrix(service(s), service(s+1));
        end

        if sum(demands(service)) <= W_B && flight_dist <= U_B
            offspring.drone_B_tasks{end+1, 1} = launch;
            offspring.drone_B_tasks{end, 2} = service;
            offspring.drone_B_tasks{end, 3} = recovery;
            offspring.drone_A_tasks(k,:) = [];
        end
    elseif ~isempty(offspring.drone_B_tasks) && rand() < 0.5
        % 尝试将B任务切换为A
        k = randi(size(offspring.drone_B_tasks, 1));
        service = offspring.drone_B_tasks{k,2};
        if iscell(service), service = cell2mat(service); end
        launch = offspring.drone_B_tasks{k,1};
        recovery = offspring.drone_B_tasks{k,3};

        flight_dist = dist_matrix(launch, service(1)) + dist_matrix(service(end), recovery);
        for s = 1:length(service)-1
            flight_dist = flight_dist + dist_matrix(service(s), service(s+1));
        end

        if sum(demands(service)) <= W_A && flight_dist <= U_A
            offspring.drone_A_tasks{end+1, 1} = launch;
            offspring.drone_A_tasks{end, 2} = service;
            offspring.drone_A_tasks{end, 3} = recovery;
            offspring.drone_B_tasks(k,:) = [];
        end
    end
end

% 2b. 车辆节点转为无人机服务 (新增变异算子)
if rand() < 0.5
    vr_inner = offspring.vehicle_route(2:end-1);
    if ~isempty(vr_inner) && length(vr_inner) >= 2
        candidate_idx = randi(length(vr_inner));
        candidate_node = vr_inner(candidate_idx);
        assigned = false;
        if demands(candidate_node) <= W_B
            launch_idx = max(1, candidate_idx-1);
            recovery_idx = min(length(vr_inner), candidate_idx+1);
            if recovery_idx > launch_idx
                launch_node = vr_inner(launch_idx);
                recovery_node = vr_inner(recovery_idx);
                flight_dist = dist_matrix(launch_node, candidate_node) + dist_matrix(candidate_node, recovery_node);
                if flight_dist <= U_B
                    offspring.drone_B_tasks{end+1, 1} = launch_node;
                    offspring.drone_B_tasks{end, 2} = candidate_node;
                    offspring.drone_B_tasks{end, 3} = recovery_node;
                    assigned = true;
                end
            end
        end
        if ~assigned && demands(candidate_node) <= W_A
            launch_idx = max(1, candidate_idx-1);
            recovery_idx = min(length(vr_inner), candidate_idx+1);
            if recovery_idx > launch_idx
                launch_node = vr_inner(launch_idx);
                recovery_node = vr_inner(recovery_idx);
                flight_dist = dist_matrix(launch_node, candidate_node) + dist_matrix(candidate_node, recovery_node);
                if flight_dist <= U_A
                    offspring.drone_A_tasks{end+1, 1} = launch_node;
                    offspring.drone_A_tasks{end, 2} = candidate_node;
                    offspring.drone_A_tasks{end, 3} = recovery_node;
                    assigned = true;
                end
            end
        end
        % 如果成功分配给无人机, 从车辆路径中移除该节点
        if assigned
            vr_inner(candidate_idx) = [];
            offspring.vehicle_route = [warehouse, vr_inner, warehouse];
        end
    end
end

% 2c. 服务节点调整 (概率0.4新增, 0.4删除)
if rand() < 0.4
    % 新增节点
    if ~isempty(offspring.drone_A_tasks) || ~isempty(offspring.drone_B_tasks)
        % 找一个未服务的节点
        drone_served = [];
        for k = 1:size(offspring.drone_A_tasks, 1)
            s = offspring.drone_A_tasks{k,2};
            if iscell(s), s = cell2mat(s); end
            drone_served = [drone_served, s];
        end
        for k = 1:size(offspring.drone_B_tasks, 1)
            s = offspring.drone_B_tasks{k,2};
            if iscell(s), s = cell2mat(s); end
            drone_served = [drone_served, s];
        end

        vehicle_served = offspring.vehicle_route(2:end-1);
        unserved = setdiff(2:n_nodes, [drone_served, vehicle_served]);

        if ~isempty(unserved) && ~isempty(offspring.drone_B_tasks)
            new_node = unserved(randi(length(unserved)));
            k_task = randi(size(offspring.drone_B_tasks, 1));
            s = offspring.drone_B_tasks{k_task, 2};
            if iscell(s), s = cell2mat(s); end
            s_new = [s, new_node];

            launch = offspring.drone_B_tasks{k_task, 1};
            recovery = offspring.drone_B_tasks{k_task, 3};
            flight_dist = dist_matrix(launch, s_new(1)) + dist_matrix(s_new(end), recovery);
            for si = 1:length(s_new)-1
                flight_dist = flight_dist + dist_matrix(s_new(si), s_new(si+1));
            end

            if sum(demands(s_new)) <= W_B && flight_dist <= U_B
                offspring.drone_B_tasks{k_task, 2} = s_new;
            end
        end
    end
elseif rand() < 0.4
    % 删除最远节点
    if ~isempty(offspring.drone_A_tasks)
        k = randi(size(offspring.drone_A_tasks, 1));
        s = offspring.drone_A_tasks{k,2};
        if iscell(s), s = cell2mat(s); end
        if length(s) > 1
            launch = offspring.drone_A_tasks{k,1};
            dists = dist_matrix(launch, s);
            [~, idx_remove] = max(dists);
            s(idx_remove) = [];
            offspring.drone_A_tasks{k,2} = s;
        end
    elseif ~isempty(offspring.drone_B_tasks)
        k = randi(size(offspring.drone_B_tasks, 1));
        s = offspring.drone_B_tasks{k,2};
        if iscell(s), s = cell2mat(s); end
        if length(s) > 1
            launch = offspring.drone_B_tasks{k,1};
            dists = dist_matrix(launch, s);
            [~, idx_remove] = max(dists);
            s(idx_remove) = [];
            offspring.drone_B_tasks{k,2} = s;
        end
    end
end

% 3. 修复
offspring = resolve_drone_conflicts(offspring, n_nodes, warehouse);
end

%% ===========================================================================
%%                 局部搜索优化
%% ===========================================================================
function chromosome = local_search_optimize(chromosome, demands, dist_matrix, ...
    V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
    service_D_A, service_D_B, At_max, M, ...
    n_nodes, warehouse, iterations)

for iter = 1:iterations
    % 车辆路径变邻域搜索
    vr = chromosome.vehicle_route;
    improved = false;

    % 2-opt搜索
    best_vr = vr;
    best_dist = path_length(vr, dist_matrix);

    for i = 2:length(vr)-3
        for j = i+1:length(vr)-2
            new_vr = vr;
            new_vr(i:j) = fliplr(new_vr(i:j));
            new_dist = path_length(new_vr, dist_matrix);
            if new_dist < best_dist
                best_vr = new_vr;
                best_dist = new_dist;
                improved = true;
            end
        end
    end

    if improved
        chromosome.vehicle_route = best_vr;
    end

    % 无人机任务优化 - 贪心重分配（仅当改进才接受）
    if rand() < 0.7
        if length(chromosome.vehicle_route) >= 3
            % 保存当前方案用于比较
            old_drone_A = chromosome.drone_A_tasks;
            old_drone_B = chromosome.drone_B_tasks;
            old_vr = chromosome.vehicle_route;
            
            % 计算当前方案适应度
            old_fitness = evaluate_fitness(chromosome, dist_matrix, demands, n_nodes, ...
                V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
                service_D_A, service_D_B, At_max, M);

            [drone_A, drone_B, new_vr] = greedy_drone_assignment(...
                chromosome.vehicle_route, demands, dist_matrix, ...
                W_A, W_B, U_A, U_B);

            % 计算新方案适应度，仅当改进才接受
            if ~isempty(drone_A) || ~isempty(drone_B)
                test_chrom.vehicle_route = new_vr;
                test_chrom.drone_A_tasks = drone_A;
                test_chrom.drone_B_tasks = drone_B;
                new_fitness = evaluate_fitness(test_chrom, dist_matrix, demands, n_nodes, ...
                    V_T, service_T, V_A, V_B, W_A, W_B, U_A, U_B, ...
                    service_D_A, service_D_B, At_max, M);
                
                if new_fitness > old_fitness
                    chromosome.drone_A_tasks = drone_A;
                    chromosome.drone_B_tasks = drone_B;
                    chromosome.vehicle_route = new_vr;
                else
                    % 恢复原方案
                    chromosome.drone_A_tasks = old_drone_A;
                    chromosome.drone_B_tasks = old_drone_B;
                    chromosome.vehicle_route = old_vr;
                end
            end
        end
    end

    % 修复
    chromosome = resolve_drone_conflicts(chromosome, n_nodes, warehouse);
end
end

%% ===========================================================================
%%                 辅助函数
%% ===========================================================================
function d = path_length(route, dist_matrix)
d = 0;
for i = 1:length(route)-1
    d = d + dist_matrix(route(i), route(i+1));
end
end

% 平均绝对偏差
function diversity = compute_diversity(fitness)
N = length(fitness);
if N <= 1
    diversity = 0;
    return;
end
mean_fit = mean(fitness);
sum_abs = sum(abs(fitness - mean_fit));
diversity = sum_abs / N;  % 论文公式
end

%% ===========================================================================
%%               绘制配送路径图
%% ===========================================================================
function plot_solution(vehicle_route, drone_tasks_A, drone_tasks_B, coords, title_str)
hold on;

% 绘制节点
n_nodes = size(coords, 1);

% 仓库 (红色五角星)
scatter(coords(1,1), coords(1,2), 200, 'r', 'p', 'filled', 'MarkerEdgeColor', 'k');
text(coords(1,1)+1, coords(1,2)+1, '仓库(1)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');

% 其他节点
for i = 2:n_nodes
    scatter(coords(i,1), coords(i,2), 80, 'b', 'o', 'filled');
    text(coords(i,1)+1, coords(i,2)-1, num2str(i), 'FontSize', 9);
end

% 车辆路径 (黑色实线)
for i = 1:length(vehicle_route)-1
    from_node = vehicle_route(i);
    to_node = vehicle_route(i+1);
    plot([coords(from_node,1), coords(to_node,1)], ...
        [coords(from_node,2), coords(to_node,2)], ...
        'k-', 'LineWidth', 2.5);
    % 添加箭头
    mid_x = (coords(from_node,1) + coords(to_node,1)) / 2;
    mid_y = (coords(from_node,2) + coords(to_node,2)) / 2;
    dx = coords(to_node,1) - coords(from_node,1);
    dy = coords(to_node,2) - coords(from_node,2);
    len = sqrt(dx^2 + dy^2);
    if len > 0
        quiver(mid_x - dx*0.05, mid_y - dy*0.05, dx*0.1, dy*0.1, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    end
end

% 无人机A路径 (红色虚线)
for k = 1:size(drone_tasks_A, 1)
    launch = drone_tasks_A{k,1};
    service = drone_tasks_A{k,2};
    recovery = drone_tasks_A{k,3};
    if iscell(service), service = cell2mat(service); end

    % 发射点到第一个服务点
    plot_drone_path(coords, launch, service(1), 'r--', 1.5);
    % 服务点之间
    for s = 1:length(service)-1
        plot_drone_path(coords, service(s), service(s+1), 'r--', 1.5);
    end
    % 最后一个服务点到回收点
    plot_drone_path(coords, service(end), recovery, 'r--', 1.5);

    % 标记无人机路径标注
    text(mean([coords(launch,1), coords(service(1),1)]), ...
        mean([coords(launch,2), coords(service(1),2)])-2, ...
        sprintf('A%d', k), 'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
end

% 无人机B路径 (绿色虚线)
for k = 1:size(drone_tasks_B, 1)
    launch = drone_tasks_B{k,1};
    service = drone_tasks_B{k,2};
    recovery = drone_tasks_B{k,3};
    if iscell(service), service = cell2mat(service); end

    plot_drone_path(coords, launch, service(1), 'g--', 1.5);
    for s = 1:length(service)-1
        plot_drone_path(coords, service(s), service(s+1), 'g--', 1.5);
    end
    plot_drone_path(coords, service(end), recovery, 'g--', 1.5);

    text(mean([coords(launch,1), coords(service(1),1)])+1, ...
        mean([coords(launch,2), coords(service(1),2)])-1, ...
        sprintf('B%d', k), 'Color', [0 0.6 0], 'FontSize', 8, 'FontWeight', 'bold');
end

% 图例
h1 = plot(NaN, NaN, 'k-', 'LineWidth', 2.5);
h2 = plot(NaN, NaN, 'r--', 'LineWidth', 1.5);
h3 = plot(NaN, NaN, 'g--', 'LineWidth', 1.5);
legend([h1, h2, h3], {'车辆路径', '无人机A路径', '无人机B路径'}, 'Location', 'best');

title(title_str);
xlabel('X坐标 (km)');
ylabel('Y坐标 (km)');
grid on;
axis equal;
hold off;
end

function plot_drone_path(coords, from_node, to_node, style, width)
x = [coords(from_node,1), coords(to_node,1)];
y = [coords(from_node,2), coords(to_node,2)];
plot(x, y, style, 'LineWidth', width);
end