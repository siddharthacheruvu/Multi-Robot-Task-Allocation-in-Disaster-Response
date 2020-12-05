%% Multi-Robot Task Allocation
% This version of the code is for a static enviroment and
% the solution approach is based on Simulated Annealing method for a single
% robot.

%% Initializing the parameters and figure
clear;
clc;
close all;
numTasks = 10;
grid_limit = 100;
coords = grid_limit*rand(numTasks,2);
% Subplot1
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,2,1)
coord_comp = [coords;coords(1,:)];
plot(coord_comp(:,1), coord_comp(:,2),'b','LineWidth',1.5); hold on;
plot(coords(:,1),coords(:,2),'s','MarkerSize',10, 'MarkerEdgeColor','red',...
    'MarkerFaceColor',[1 .6 .6]);
legend('Robot Path', 'Task Sites');
title('Path');
xlabel('x');
ylabel('y');
% axis equal;
grid on;
xlim([0 grid_limit]);
ylim([0 grid_limit]);
xticks([0:grid_limit/10:grid_limit]);
yticks([0:grid_limit/10:grid_limit]);

%% Simulated Annealing
cost0 = get_total_distance(coords);
global_min = cost0;
opt_coords = coords;
T = 30;
T_init = T;
c_init = cost0;
alpha = 0.99;
i = 1;
costMat = [];
probMat = [];
tempMat = [];
nSteps = 500;

for i = 1:nSteps
    costMat(end+1,:) = cost0;
    tempMat(end+1,:) = T;
    tempcost = cost0;
    % Subplot2
    subplot(2,2,2)
    plot([1:1:i], costMat,'b','LineWidth',1.5);
    title('Total Length(Cost)');
    xlabel('Step');
    ylabel('Length');
    xlim([0 nSteps]);
    ylim([0 c_init*1.2]);
    % Subplot4
    subplot(2,2,4)
    plot([1:1:i], tempMat,'b','LineWidth',1.5); hold on;
    title('Temperature');
    xlabel('Step');
    ylabel('Temperature');
    xlim([0 nSteps]);
    ylim([0 T_init]);
    X = sprintf('%d, cost = %0.5f',i,cost0);
    T = T*alpha;
    for j = 1:500
        r = randperm(numTasks,2);
        temp = coords(r(1),:);
        coords(r(1),:) = coords(r(2),:);
        coords(r(2),:) = temp;
        cost1 = get_total_distance(coords);
        if cost1 < cost0
            cost0 = cost1;
        else
            x = rand;
            loss = cost0 - cost1;
            prob = exp(loss/T);
            if x < prob
                cost0 = cost1;
            else
                temp = coords(r(1),:);
                coords(r(1),:) = coords(r(2),:);
                coords(r(2),:) = temp;
            end
        end        
    end
    if global_min > cost0
        global_min = cost0;
        opt_coords = coords;
    end
    
    probMat(end+1,:) = prob;
    % Subplot3
    subplot(2,2,3)
    plot([1:1:i], probMat,'b.','MarkerSize',8); hold on;
    title('Acceptance Probability');
    xlabel('Step');
    ylabel('Probability');
    xlim([0 nSteps]);
    ylim([0 1]);
    figure(1)
    subplot(2,2,1);
    
    % Subplot 1
    coord_comp = [coords;coords(1,:)];
    plot(coord_comp(:,1), coord_comp(:,2),'b','LineWidth',1.5); hold on;
    plot(coords(:,1),coords(:,2),'s','MarkerSize',10, 'MarkerEdgeColor','red',...
    'MarkerFaceColor',[1 .6 .6]);
    legend('Robot Path', 'Task Sites');
    hold off;
    title('Path');
    xlabel('x');
    ylabel('y');
%     axis equal;
    grid on;
    xlim([0 grid_limit]);
    ylim([0 grid_limit]);
    xticks([0:grid_limit/10:grid_limit]);
    yticks([0:grid_limit/10:grid_limit]);
end

%% Optimal Solution
% Subplot 1
coord_comp = [opt_coords;opt_coords(1,:)];
plot(coord_comp(:,1), coord_comp(:,2),'b','LineWidth',1.5); hold on;
plot(opt_coords(:,1),opt_coords(:,2),'s','MarkerSize',10, 'MarkerEdgeColor','red',...
'MarkerFaceColor',[1 .6 .6]);
hold off;
legend('Robot Path', 'Task Sites');
title('Path');
xlabel('x');
ylabel('y');
%     axis equal;
grid on;
xlim([0 grid_limit]);
ylim([0 grid_limit]);
xticks([0:grid_limit/10:grid_limit]);
yticks([0:grid_limit/10:grid_limit]);
X = sprintf('Optimal Path Length is %.2f',global_min);
disp(X);
f = msgbox('Optimal Allocation Found!');
function [totaldist] = get_total_distance(coords)
    totaldist = 0;
    for i = 1:length(coords)-1
        totaldist = totaldist + norm(coords(i+1,:) - coords(i,:));
    end
    totaldist = totaldist + norm(coords(1,:) - coords(end,:));
end