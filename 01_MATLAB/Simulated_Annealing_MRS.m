%% Multi-Robot Task Allocation
% This version of the code is for a static enviroment and
% the solution approach is based on Simulated Annealing method for multiple
% robots.

%% Initializing the parameters and figure
clear;
clc;
close all;
numTasks = 10;
grid_limit = 100;
numRobots = 3;
coords = grid_limit*rand(numTasks,2);
Distr = [3 4 3];
ind_Distr = randperm(numTasks,numTasks);
coord_temp = zeros(numTasks,2);
for i = 1:numTasks
    coord_temp(i,:) = coords(ind_Distr(i),:);
end
st1 = [5 5];
st2 = [5 5];
st3 = [5 5];
coords1 = [st1;coord_temp(1:Distr(1),:)];
coords2 = [st2;coord_temp(Distr(1)+1:Distr(1)+Distr(2),:)];
coords3 = [st3;coord_temp(Distr(1)+Distr(2)+1:Distr(1)+Distr(2)+Distr(3),:)];
% figure(1);
% Subplot1
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,2,1)
coord_comp1 = [coords1;coords1(1,:)];
plot(coord_comp1(:,1), coord_comp1(:,2),'r','LineWidth',1.5); hold on;
coord_comp2 = [coords2;coords2(1,:)];
plot(coord_comp2(:,1), coord_comp2(:,2),'b','LineWidth',1.5); hold on;
coord_comp3 = [coords3;coords3(1,:)];
plot(coord_comp3(:,1), coord_comp3(:,2),'g','LineWidth',1.5); hold on;
plot(coords(:,1),coords(:,2),'s','MarkerSize',10, 'MarkerEdgeColor','red',...
    'MarkerFaceColor',[1 .6 .6]);
legend('Robot 1', 'Robot 2', 'Robot 3', 'Task Sites');
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
cost0_1 = get_total_distance(coords1);
cost0_2 = get_total_distance(coords2);
cost0_3 = get_total_distance(coords3);
cost0 = cost0_1 + cost0_2 + cost0_3;
global_min = cost0;
global_ind = ind_Distr;
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
        temp = ind_Distr(r(1));
        ind_Distr(r(1)) = ind_Distr(r(2));
        ind_Distr(r(2)) = temp;
        for k = 1:numTasks
            coord_temp(k,:) = coords(ind_Distr(k),:);
        end
        coords1 = [st1;coord_temp(1:Distr(1),:)];
        coords2 = [st2;coord_temp(Distr(1)+1:Distr(1)+Distr(2),:)];
        coords3 = [st3;coord_temp(Distr(1)+Distr(2)+1:Distr(1)+Distr(2)+Distr(3),:)];      
        cost1_1 = get_total_distance(coords1);
        cost1_2 = get_total_distance(coords2);
        cost1_3 = get_total_distance(coords3);
        cost1 = cost1_1 + cost1_2 + cost1_3;
        if cost1 < cost0
            cost0 = cost1;
        else
            x = rand;
            loss = cost0 - cost1;
            prob = exp(loss/T);
            if x < prob
                cost0 = cost1;
            else
                temp = ind_Distr(r(1));
                ind_Distr(r(1)) = ind_Distr(r(2));
                ind_Distr(r(2)) = temp;
                for k = 1:numTasks
                    coord_temp(k,:) = coords(ind_Distr(k),:);
                end
                coords1 = [st1;coord_temp(1:Distr(1),:)];
                coords2 = [st2;coord_temp(Distr(1)+1:Distr(1)+Distr(2),:)];
                coords3 = [st3;coord_temp(Distr(1)+Distr(2)+1:Distr(1)+Distr(2)+Distr(3),:)]; 
            end
        end        
    end
    if cost0 < global_min
        global_min = cost0;
        global_ind = ind_Distr;
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
    coord_comp1 = [coords1;coords1(1,:)];
    plot(coord_comp1(:,1), coord_comp1(:,2),'r','LineWidth',1.5); hold on;
    coord_comp2 = [coords2;coords2(1,:)];
    plot(coord_comp2(:,1), coord_comp2(:,2),'b','LineWidth',1.5); hold on;
    coord_comp3 = [coords3;coords3(1,:)];
    plot(coord_comp3(:,1), coord_comp3(:,2),'g','LineWidth',1.5); hold on;
    plot(coords(:,1),coords(:,2),'s','MarkerSize',10, 'MarkerEdgeColor','red',...
    'MarkerFaceColor',[1 .6 .6]);
    legend('Robot 1', 'Robot 2', 'Robot 3', 'Task Sites');
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

for i = 1:numTasks
    coord_temp(i,:) = coords(global_ind(i),:);
end

%% Plotting Optimal Solution
coords1 = [st1;coord_temp(1:Distr(1),:)];
coords2 = [st2;coord_temp(Distr(1)+1:Distr(1)+Distr(2),:)];
coords3 = [st3;coord_temp(Distr(1)+Distr(2)+1:Distr(1)+Distr(2)+Distr(3),:)];
subplot(2,2,1)
coord_comp1 = [coords1;coords1(1,:)];
plot(coord_comp1(:,1), coord_comp1(:,2),'r','LineWidth',1.5); hold on;
coord_comp2 = [coords2;coords2(1,:)];
plot(coord_comp2(:,1), coord_comp2(:,2),'b','LineWidth',1.5); hold on;
coord_comp3 = [coords3;coords3(1,:)];
plot(coord_comp3(:,1), coord_comp3(:,2),'g','LineWidth',1.5); hold on;
plot(coords(:,1),coords(:,2),'s','MarkerSize',10, 'MarkerEdgeColor','red',...
    'MarkerFaceColor',[1 .6 .6]);
legend('Robot 1', 'Robot 2', 'Robot 3', 'Task Sites');
title('Path');
xlabel('x');
ylabel('y');
% axis equal;
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