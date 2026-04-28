function [dataStore] = prmPlanner(Robot, maxTime)

    if nargin < 2
        maxTime = 500;
    end
    
    global dataStore;
    dataStore = struct('truthPose', [],...
                       'odometry', [], ...
                       'rsdepth', [], ...
                       'bump', [], ...
                       'beacon', []);
    
    noRobotCount = 0;
    [noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % ---------------------------
    % Load map
    % ---------------------------
    A = load("C:\Users\danny\Documents\AMR\Project\3credits_practice\map1_3credits.mat");
    
    sim_bound = [-5 -4 5 4];
    
    walls = A.map;
    stayAwayPoints = A.stayAwayPoints;
    
    % ---------------------------
    % Build polygon obstacles
    % ---------------------------
    obstacles = {};
    
    obs_radius = 0.05;
    
    for i = 1:size(stayAwayPoints,1)
    
        x = stayAwayPoints(i,1);
        y = stayAwayPoints(i,2);
    
        obstacles{end+1} = [
            x-obs_radius y-obs_radius;
            x+obs_radius y-obs_radius;
            x+obs_radius y+obs_radius;
            x-obs_radius y+obs_radius
        ];
    end
    
    % ---------------------------
    % Robot state
    % ---------------------------
    start = [dataStore.truthPose(1,2), dataStore.truthPose(1,3)];
    waypt = orderWaypoints([A.waypoints; A.ECwaypoints], start);
    

    % =========================================================
    % BUILD PRM 
    % =========================================================
    n_PRM = 150;
    robotRadius = 0.2;
    
    PRM = buildPRM(obstacles, walls, sim_bound, start, waypt(1,:), ...
                   n_PRM, robotRadius, stayAwayPoints, @haltonSample);
    

    
    % =========================================================
    % EXECUTION LOOP
    % =========================================================
    
    closeEnough = 0.2;
    epsilon = 0.2;
    
    gotopt = 1;
    numWayPt = size(waypt,1);
    wayptVisited = [];
    
    tic
    while toc < maxTime
        
        % Save the current state of the robot
        [noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
        currentPose = dataStore.truthPose(end,:);
        q_current = [currentPose(2), currentPose(3)];

    
        if gotopt > numWayPt
            SetFwdVelAngVelCreate(Robot,0,0);
            break;
        end

        % Order waypoints based on current state
        waypt = orderWaypoints(waypt(2:end, :), q_current);
    
        q_goal = waypt(1,:);
    
        % ---------------------------
        % PLAN PATH USING PRM
        % ---------------------------
        path = findPath(PRM, q_current, q_goal, obstacles, walls, stayAwayPoints, robotRadius);

        % OPTIONAL PLOTTING OF PATH ALONG PRM
        % figure;
        % plotPRM(PRM, walls, sim_bound);
        % 
        % hold on;
        % plot(path(:,1), path(:,2), 'g', 'LineWidth', 2);
        % plot(start(1), start(2), 'go', 'MarkerSize',10,'LineWidth',2);
        % plot(waypt(1,1), waypt(1,2), 'mo', 'MarkerSize',10,'LineWidth',2);
    
        % ---------------------------
        % FOLLOW PATH
        % ---------------------------
        
        % CHANGE LED BACK TO GREEN HERE
        % SetLEDsRoomba(Robot, 3, 0, 100);

        for p = 2:size(path,1)
            
            reached = false;
    
            while ~reached && toc < maxTime
    
                [noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
                currentPose = dataStore.truthPose(end,:);
                q = [currentPose(2), currentPose(3)];
    
                dx = path(p,1) - q(1);
                dy = path(p,2) - q(2);
    
                dist = norm([dx dy]);
    
                [v,w] = feedbackLin(dx,dy,currentPose(4),epsilon);
                [v,w] = limitCmds(v,w,0.2,0.13);
    
                SetFwdVelAngVelCreate(Robot,v,w);
    
                if dist < closeEnough
                    reached = true;
                end
            end
        end

        wayptVisited = [wayptVisited; q_goal];
        % CHANGE LED TO RED HERE
        % SetLEDsRoomba(Robot, 3, 100, 100);
        
        gotopt = gotopt + 1;
    end
    
    SetFwdVelAngVelCreate(Robot,0,0);

end