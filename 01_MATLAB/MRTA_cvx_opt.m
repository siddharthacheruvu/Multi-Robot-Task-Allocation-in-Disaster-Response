%% Multi-Robot Task Allocation
% This version of the code is for a static enviroment and
% the solution is obtained using Convex Optimization solver
% NOTE: In order to run this file, 'Mobile Robotics Simulation Toolbox' is
% needed. Please install the toolbox from the 'Add-Ons' ribbon.

%% Create a multi-robot environment
clear;
close all;
global numRobots;
numRobots = 3;
global rSpeed;
global grid_limit;
grid_limit = 100;
rSpeed = 0.007*grid_limit;
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.015*grid_limit;
env.showTrajectory = true;
% env.showTrajectory = ones(1,numRobots);
env.hasWaypoints = true;
%% Initialization: Define objects (tasks), and initial poses
global numTasks;
numTasks = 10;
global waypoints;
waypoints = 10*rand(numTasks,2);
init_pose = [0;0;pi/4];
poses = repmat(init_pose,1,numRobots);
env.Poses = poses;
global sampleTime;
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:2000;        % Time array
global rStates;
rStates = zeros(numRobots,1);  % 0=idle,1=intransit,2=doingtask
global tasktracker;
tasktracker = zeros(numTasks,1); %0=notassigned,1=assigned,2=complete
global nxttaskIdx;
nxttaskIdx = zeros(numRobots,1);
global tasktimes;
tasktimes = zeros(numRobots,1);
global finTasks;
global totalLength;
totalLength = 0;
%% Simulation loop
vel = zeros(3,numRobots);
for idx=2:numel(tVec)
    if sum(tasktracker) == numTasks*2
        break;
    end
%     cointoss = rand;
%     if cointoss < 0.002
%         waypoints = [waypoints;10*rand(1,2)];
%         numTasks = numTasks + 1;
%         tasktracker = [tasktracker;0];
%     end
    env(1:numRobots,poses,waypoints)
    xlim([0 grid_limit]);   % Without this, axis resizing can slow things down
    ylim([0 grid_limit]);
    grid on;
    xticks(0:grid_limit/10:grid_limit);yticks(0:grid_limit/10:grid_limit);
    
    for rIdx=1:numRobots
        % Using the MRTA controller to determine next pose of the robots
        v = MRTAController(poses,rIdx);
        vel(:,rIdx) = v;
    end   
    % Discrete integration of pose
    poses = poses + vel*sampleTime;    
end
X = sprintf('All robots returning to base station.');
disp(X);
for rIdx = 1:numRobots
    dvec = [0;0]-poses(1:2,rIdx);
    totalLength = totalLength + norm(dvec);    
end

while max(vecnorm(poses(1:2,:))) > 10^-1 
    for rIdx = 1:numRobots
        dvec = [0;0]-poses(1:2,rIdx);
        v_vec = dvec/norm(dvec)*rSpeed;
        angle = (atan2(dvec(2),dvec(1)) - poses(3,rIdx))/sampleTime;
        vel(:,rIdx) = [v_vec;angle];                         
    end    
    poses = poses + vel*sampleTime;
    env(1:numRobots,poses,waypoints)
    xlim([0 grid_limit]);   % Without this, axis resizing can slow things down
    ylim([0 grid_limit]);
    grid on;
    xticks(0:grid_limit/10:grid_limit);yticks(0:grid_limit/10:grid_limit);
end
X = sprintf('Total Path Length is %.2f',totalLength);
disp(X);
f = msgbox('All the task are completed!');


%% Helper Function
function vel = MRTAController(poses,rIdx)
    % Fetch required global variables
    global rStates;
    global waypoints;
    global tasktracker;
    global rSpeed;
    global nxttaskIdx;
    global tasktimes;
    global sampleTime;
    global finTasks;
    global totalLength;
    global grid_limit;
    
    % Unpack the robot's pose and state
    pose = poses(:,rIdx);
    state = rStates(rIdx);
        
    % Controller Logic
    if state == 0
        if isempty(find(tasktracker==0))
            % Do nothing.
            vel = [0;0;0];
            
        else
            nxttsk = 0;
            pendtskIDs = find(tasktracker==0);
            cMat = zeros(size(poses,2),length(pendtskIDs));
            v_vec = [0;0];
            for i = 1:size(poses,2)
                for j = 1:length(pendtskIDs)
                    if i == rIdx
                        dvec = waypoints(pendtskIDs(j),:)'-poses(1:2,i);
                    else
                        if nxttaskIdx(i) == 0
                            dvec = waypoints(pendtskIDs(j),:)'-poses(1:2,i);
                        else
                            dvec = waypoints(pendtskIDs(j),:)'-waypoints(nxttaskIdx(i),:)';
                        end
                    end
                    tdist = norm(dvec);
                    cMat(i,j) = tdist;
                end
            end
            
            if length(pendtskIDs) < size(poses,2)
                cMat = [cMat 9999*ones(size(poses,2),size(poses,2)-length(pendtskIDs))];
            elseif length(pendtskIDs) > size(poses,2)
                cMat = [cMat;9999*ones(length(pendtskIDs)-size(poses,2),length(pendtskIDs))];
            else
                % Do Nothing
            end
            c = reshape(cMat',1,[]);            
            Aeq = zeros(2*size(cMat,1),length(c));
            beq = ones(2*size(cMat,1),1);

            for k = 1:size(cMat,1)
                for p = 1:size(cMat,1)
                   Aeq(k,couple2Index(p,k,size(cMat,1))) = 1;
                   Aeq(k+size(cMat,1),couple2Index(k,p,size(cMat,1))) = 1;
                end
            end
            options = optimoptions('linprog','Algorithm','dual-simplex','Display','iter');
            xopt = linprog(c,[],[],Aeq,beq,zeros(length(c),1),[],[],options);
            optind = find(xopt==1);
            assignments = index2Couple(optind,size(cMat,1));
            tskalloc = assignments(:,2);
            if cMat(rIdx,tskalloc(rIdx)) ~= 9999
                nxttsk = pendtskIDs(tskalloc(rIdx));
                tasktracker(nxttsk) = 1;
                nxttaskIdx(rIdx) = nxttsk;
                X = sprintf('Robot %d takes up task %d',rIdx,nxttsk);
                disp(X);
                rStates(rIdx) = 1;
                dvec = waypoints(nxttsk,:)'-pose(1:2);
                v_vec = dvec/norm(dvec)*rSpeed;
                angle = (atan2(dvec(2),dvec(1)) - pose(3))/sampleTime;
                vel = [v_vec;angle];
                totalLength = totalLength + norm(dvec);
            else
                nxttsk = 0;
                nxttaskIdx(rIdx) = 0;
                vel = [v_vec;0];
                
            end
        end
    elseif state == 1
        dvec = waypoints(nxttaskIdx(rIdx),:)'-pose(1:2);
        if norm(dvec) < 10^(-3)*grid_limit
            rStates(rIdx) = 2;
            vel = [0;0;0];
        else
            v_vec = dvec/norm(dvec)*rSpeed;
            vel = [v_vec;0];
        end 
    elseif state == 2
        if tasktimes(rIdx) == sampleTime*500
            rStates(rIdx) = 0;
            tasktimes(rIdx) = 0;
            tasktracker(nxttaskIdx(rIdx)) = 2;
            vel = [0;0;0];
%             finTasks(end+1,:) = waypoints(nxttaskIdx(rIdx),:);
%             plot(finTasks(end,1),finTasks(end,2),'o','MarkerEdgeColor','green','MarkerSize',8);
        else
            tasktimes(rIdx) = tasktimes(rIdx) + 1;
            vel = [0;0;0];
        end
        
    end
            

end

function [index] = couple2Index(i,j,N)
    index = N*(i-1) + j;
end

function [couple] = index2Couple(ind,N)
    couple(:,1) = ceil(ind/N);
    couple(:,2) = ind - N*(couple(:,1)-1);
end

