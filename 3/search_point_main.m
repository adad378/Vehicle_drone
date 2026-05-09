%%第三章
%在30个受灾点的情况下，选择最优投放点位置
%支持 c1、c2、c3 三个算例，切换只需修改 case_name
clear;clc;close all;

%%=========== 选择算例=================
case_name = 'c1';  % 可选：c1 / c2 / c3
fprintf('======== 当前运行：%s 算例 ========\n', case_name);

%%===========设置初始参数=================
c_i=10; %投放点的单位综合成本
a=5; %运营无人机的单位成本
beta=2; %无人机搭载物资的单位配送成本
R_i=1e6; %投放点i的物资承载量
W=50; %无人机的最大载重
v=80; %无人机的飞行速度均值
d_max=100; %搭载物资后无人机的最大飞行距离
gamma=0.3; %受灾点可接受时间满意度的最低要求
t_E=0.2; %受灾点最短可接受的完成配送时间
t_L=2.0; %受灾点最长可接受的完成配送时间

%%=========== 三个算例数据 =================
disaster_points_c1=[
    1   52  75  10
    2   45  70  30
    3   62  69  10
    4   60  66  10
    5   42  65  10
    6   35  69  10
    7   25  85  20
    8   22  75  30
    9   22  85  10
    10  20  80  40
    11  28  52  20
    12  14  66  10
    13  25  50  10
    14  22  66  40
    15  8   62  10
    16  5   35  10
    17  5   45  10
    18  2   40  20
    19  0   40  30
    20  0   45  20
    21  3   22  10
    22  3   30  10
    23  34  25  30
    24  30  35  10
    25  36  40  10
    26  42  15  10
    27  40  5   30
    28  38  15  40
    29  38  5   30
    30  38  10  10
];
disaster_points_c2 = [
    1   41  49  10
    2   35  17  7
    3   55  45  13
    4   55  20  19
    5   15  30  26
    6   25  30  3
    7   20  50  5
    8   10  43  9
    9   55  60  16
    10  30  60  16
    11  20  65  12
    12  50  35  19
    13  30  25  23
    14  15  10  20
    15  30  5   8
    16  10  20  19
    17  5   30  2
    18  20  40  12
    19  15  60  17
    20  45  65  9
    21  45  20  11
    22  45  10  18
    23  55  5   29
    24  65  35  3
    25  65  20  6
    26  45  30  17
    27  35  40  16
    28  41  37  16
    29  64  42  9
    30  40  60  21
];
disaster_points_c3 = [
    1   25  85  20
    2   22  75  30
    3   22  85  10
    4   20  80  40
    5   20  85  20
    6   5   35  10
    7   5   45  10
    8   2   40  20
    9   0   40  20
    10  0   45  20
    11  38  15  10
    12  35  5   20
    13  95  30  30
    14  95  35  20
    15  92  30  10
    16  88  35  20
    17  87  30  10
    18  85  25  10
    19  85  35  30
    20  67  85  20
    21  58  75  20
    22  55  80  10
    23  55  85  20
    24  55  82  10
    25  20  82  10
    26  35  69  23
    27  65  55  14
    28  63  65  8
    29  2   60  5
    30  20  20  8
];

%%=========== 自动加载选中的算例 =================
switch case_name
    case 'c1'
        disaster_points = disaster_points_c1;
    case 'c2'
        disaster_points = disaster_points_c2;
    case 'c3'
        disaster_points = disaster_points_c3;
    otherwise
        error('请输入正确的算例名称：c1/c2/c3');
end

coords_disaster = disaster_points(:,2:3);
demands = disaster_points(:,4);
N_disaster = size(disaster_points,1);

%%===========随机生成备选点，和声搜索算法优化=================
K=12; %备选点数量
rng('shuffle');
[cluster_idx,center_coords]=kmeans(coords_disaster,K);
N_center_coords=size(center_coords,1);
fprintf('备选点数量:%d\n',N_center_coords);

%计算距离矩阵
dist_IJ=zeros(N_center_coords,N_disaster);
for i=1:N_center_coords
    for j=1:N_disaster
        dist_IJ(i,j)=norm(center_coords(i,:)-coords_disaster(j,:));
    end
end

%%=====================IHS算法参数======================
N_HMS=100;
T_max=200;
HMCR_min=0.8;
HMCR_max=0.9;
PAR_min=0.01;
PAR_max=0.2;

%%===========IHS主函数=================
HM_Memory=randi([0,1],N_HMS,N_center_coords);
fitness=zeros(N_HMS,1);
for i=1:N_HMS
    fitness(i)=calculate_fitness(HM_Memory(i,:),...
        dist_IJ,demands,c_i,a,beta,R_i,W,v,d_max,gamma,t_E,t_L);
end
[best_fitness,best_idx]=min(fitness);
best_x=HM_Memory(best_idx,:);
best_fitness_history=zeros(T_max,1);

for t=1:T_max
    HMCR=HMCR_max*(HMCR_min/HMCR_max)^(t/T_max);
    PAR=PAR_min+(PAR_max-PAR_min)*(t/T_max);

    new_x=zeros(1,N_center_coords);
    for j=1:N_center_coords
        if rand()<HMCR
            new_x(j)=HM_Memory(randi(N_HMS),j);
        else
            new_x(j)=randi([0,1]);
        end
        if rand()<PAR
            new_x(j)=1-new_x(j);
        end
    end
    
    new_fitness=calculate_fitness(new_x,...
        dist_IJ,demands,c_i,a,beta,R_i,W,v,d_max,gamma,t_E,t_L);

    [worst_fitness,worst_idx]=max(fitness);
    if new_fitness<worst_fitness
        HM_Memory(worst_idx,:)=new_x;
        fitness(worst_idx)=new_fitness;
        if new_fitness<best_fitness
            best_fitness=new_fitness;
            best_x=new_x;
        end
    end

    best_fitness_history(t)=best_fitness;
    if mod(t,10)==0
        fprintf('迭代次数:%d, 当前最优适应度:%.4f\n',t,best_fitness);
    end
end
fprintf('优化完成! 最优适应度:%.4f\n',best_fitness);

%%===========结果=================
best_x_1=find(best_x==1);
N_best_x_1=length(best_x_1);
fprintf('选中的投放点地址索引:%s\n',num2str(best_x_1));
fprintf('选中的投放点数量:%d\n',N_best_x_1);

[~,y_idx]=calculate_fitness(best_x,...
    dist_IJ,demands,c_i,a,beta,R_i,W,v,d_max,gamma,t_E,t_L);

coverage=cell(N_best_x_1,1);
for k=1:N_best_x_1
    coverage{k}=find(y_idx(best_x_1(k),:)==1);
end

%%===========画图=================
figure;
hold on;
scatter(coords_disaster(:,1),coords_disaster(:,2),30,'b','filled');
scatter(center_coords(:,1),center_coords(:,2),30,'g','s','filled');
scatter(center_coords(best_x_1,1),center_coords(best_x_1,2),100,'r','p','filled');

for k=1:N_best_x_1
    for j=coverage{k}
        plot([center_coords(best_x_1(k),1),coords_disaster(j,1)],...
            [center_coords(best_x_1(k),2),coords_disaster(j,2)],'k--');
    end
end

for i=1:N_disaster
    text(coords_disaster(i,1)+0.5, coords_disaster(i,2)+0.5, num2str(i), 'FontSize', 8);
end
for k=1:N_best_x_1
    text(center_coords(best_x_1(k),1)+0.5, center_coords(best_x_1(k),2)+0.5, ...
        ['P', num2str(best_x_1(k))], 'FontSize', 8, 'Color', 'r');
end

xlabel('X坐标'); ylabel('Y坐标');
title([case_name,' 算例投放点选址结果']);
legend('受灾点','备选点','投放点','覆盖关系');
grid on; axis equal; hold off;

%% ================= 目标函数 =================
function [cost,y_idx] = calculate_fitness(x,dist_IJ,demands,...
    c_i,a,beta,R_i,W,v,d_max,gamma,t_E,t_L)

N_candidate = size(dist_IJ, 1);
N_disaster = size(dist_IJ, 2);
y_idx = zeros(N_candidate, N_disaster);

drop_idx = find(x == 1);
num_drops = length(drop_idx);

if num_drops == 0
    cost = 1e10;
    return;
end

y = zeros(N_candidate, N_disaster);

for j = 1:N_disaster
    d_to_drops = dist_IJ(drop_idx, j);
    [~, order] = sort(d_to_drops);
    assigned = false;
    for o = 1:num_drops
        i = drop_idx(order(o));
        dij = dist_IJ(i,j);
        if dij > d_max
            continue;
        end
        if demands(j) > W
            continue;
        end
        current_load = sum(demands .* y(i,:)');
        if current_load + demands(j) > R_i
            continue;
        end
        y(i,j) = 1;
        assigned = true;
        break;
    end
    if ~assigned
        cost = 1e10;
        y_idx = y;
        return;
    end
end

total_cost = 0;
for i = 1:N_candidate
    for j = 1:N_disaster
        if y(i,j) == 1
            Dj = demands(j);
            dij = dist_IJ(i,j);
            total_cost = total_cost + Dj * c_i + Dj * a + dij * beta;
        end
    end
end

for i = drop_idx
    covered_j = find(y(i,:) == 1);
    sum_T = 0;
    for j = covered_j
        dij = dist_IJ(i,j);
        t_ij = dij / v;
        if t_ij <= t_E
            T_val = 1;
        elseif t_ij <= t_L
            arg = 2 * pi / t_L * ((t_ij - t_E) / (t_L - t_E));
            T_val = cos(arg);
        else
            T_val = 0;
        end
        sum_T = sum_T + T_val;
    end
    if sum_T < gamma
        total_cost = total_cost + 1e6;
    end
end

cost = total_cost;
y_idx = y;
end