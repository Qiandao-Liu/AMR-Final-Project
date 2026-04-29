function result = mainLoopController(Robot, mapMatPath, opts)

    if nargin < 2 || isempty(mapMatPath)
        mapMatPath = "C:\Users\danny\Documents\AMR\Project\3credits_practice\map1_3credits.mat";
    end
    if nargin < 3
        opts = struct();
    end

    %% =========================
    % LOAD MAP
    % =========================
    mapStruct = load(mapMatPath);

    map = mapStruct.map;
    waypoints = mapStruct.waypoints;

    ECwaypoints = [];
    if isfield(mapStruct,'ECwaypoints')
        ECwaypoints = mapStruct.ECwaypoints;
    end

    stayAwayPoints = [];
    if isfield(mapStruct,'stayAwayPoints')
        stayAwayPoints = mapStruct.stayAwayPoints;
    end

    allWaypoints = [waypoints; ECwaypoints];

    sim_bound = [ ...
        min(map(:,[1,3]),[],'all'), ...
        min(map(:,[2,4]),[],'all'), ...
        max(map(:,[1,3]),[],'all'), ...
        max(map(:,[2,4]),[],'all')];

    %% =========================
    % GLOBAL DATASTORE
    % =========================
    global dataStore;
    dataStore = struct( ...
        'pfPose', [], ...
        'odometry', [], ...
        'rsdepth', [], ...
        'bump', [], ...
        'beacon', [], ...
        'visitedWaypoints', []);

    %% =========================
    % INITIAL LOCALIZATION
    % =========================
    initOpts = struct('turnRate',0.5,...
                      'loopPause',0.05,...
                      'maxTurnAngle',2*pi,...
                      'verbose',true);

    initResult = runInitialLocalizationSimulator(Robot, mapMatPath, initOpts);

    %% =========================
    % INITIALIZE PF
    % =========================
    numParticles = 500;

    state.particles = initParticlesFromPose(initResult.bestPose, numParticles);
    state.poseEstimate = initResult.bestPose;

    AngleSensorRoomba(Robot);
    DistanceSensorRoomba(Robot);

    %% =========================
    % BUILD PRM
    % =========================
    robotRadius = 0.2;
    n_PRM = 150;

    obstacles = buildObstacles(stayAwayPoints);

    PRM = buildPRM(obstacles, map, sim_bound, ...
                   state.poseEstimate(1:2)', allWaypoints(1,:), ...
                   n_PRM, robotRadius, stayAwayPoints, @haltonSample);

    %% =========================
    % PARAMETERS
    % =========================
    closeEnough = 0.2;
    epsilon = 0.15;
    maxTime = 300;

    remainingWaypoints = allWaypoints;

    currentPath = [];
    currentGoal = [];
    pathIdx = 1;

    prevPose = state.poseEstimate;
    poseEstFiltered = prevPose;

    tic;
    iter = 1;
    while toc < maxTime
    
        %% =========================
        % SENSOR UPDATE
        %% =========================
        dataStore = readAllSensors(Robot, dataStore);
    
        if isempty(dataStore.odometry) || isempty(dataStore.rsdepth)
            pause(0.05);
            continue;
        end
    
        latestOdom  = dataStore.odometry(end,2:3)';
        latestDepth = dataStore.rsdepth(end,2:end)';
        tags = RealSenseTag(Robot);
    
        %% =========================
        % PARTICLE FILTER UPDATE
        %% =========================
        [state, particlesPre] = localizeStepPF( ...
            state, latestOdom, latestDepth, map, ...
            [0.13;0], linspace(27*pi/180,-27*pi/180,10)', ...
            struct('processNoise', diag([0.004,0.004,0.003]), ...
                   'measurementNoise', 0.08*eye(10), ...
                   'tags', tags));
    
        %% =========================
        % PF HEALTH METRICS
        %% =========================
        w = particlesPre.weights;
        p = particlesPre.poses;
    
        pfSpread = sqrt(var(p(1,:)) + var(p(2,:)));
        neff = 1 / sum(w.^2);
    
        PF_GOOD = (pfSpread > 0.03) && (neff > 30);
    
        fprintf("PF spread: %.3f | Neff: %.1f\n", pfSpread, neff);
    
        %% =========================
        % STABLE POSE ESTIMATE
        %% =========================
        poseEst = state.poseEstimate;
    
        if ~exist('poseEstFiltered','var')
            poseEstFiltered = poseEst;
        end
    
        if ~PF_GOOD
            poseEst = poseEstFiltered;   % ignore unstable PF updates
        else
            poseEstFiltered = 0.8 * poseEstFiltered + 0.2 * poseEst;
            poseEst = poseEstFiltered;
        end
    
        %% =========================
        % LOGGING
        %% =========================
        dataStore.pfPose = [dataStore.pfPose; toc, poseEst'];
    
        %% =========================
        % WAYPOINT PROGRESSION
        %% =========================
        if isempty(remainingWaypoints)
            break;
        end
    
        dists = vecnorm(remainingWaypoints - poseEst(1:2)',2,2);
        [minDist, idxMin] = min(dists);
    
        if minDist < closeEnough
    
            goal = remainingWaypoints(idxMin,:);
    
            SetFwdVelAngVelCreate(Robot,0,0);
            SetLEDsRoomba(Robot,3,100,100);
            pause(1);
    
            dataStore.visitedWaypoints = [dataStore.visitedWaypoints; goal];
    
            remainingWaypoints(idxMin,:) = [];
    
            currentGoal = [];
            currentPath = [];
            pathIdx = 1;
    
            continue;
        end
    
        %% =========================
        % SELECT NEXT WAYPOINT (PRM COST)
        %% =========================
        if isempty(currentGoal)
            [currentGoal, ~] = selectNextWaypointPRM( ...
                remainingWaypoints, poseEst(1:2)', ...
                PRM, obstacles, map, stayAwayPoints, robotRadius);
        end
    
        goal = currentGoal;
    
        %% =========================
        % PATH PLANNING USING PRM
        %% =========================
        if isempty(currentPath) || mod(iter, 20) == 1
    
            currentPath = findPath(PRM, poseEst(1:2)', goal, ...
                                  obstacles, map, stayAwayPoints, robotRadius);
    
            pathIdx = 2;
    
            if isempty(currentPath)
                disp("PRM failed → retrying next cycle");
                pause(0.05);
                continue;
            end
    
            SetLEDsRoomba(Robot,3,0,100);
        end
    
        %% =========================
        % PATH FOLLOWING
        %% =========================
        lookaheadDist = 0.3;
    
        while pathIdx < size(currentPath,1) && ...
              norm(currentPath(pathIdx,:) - poseEst(1:2)') < lookaheadDist
            pathIdx = pathIdx + 1;
        end
    
        target = currentPath(pathIdx,:);
        delta = target - poseEst(1:2)';
        dist = norm(delta);
    
        %% =========================
        % CONTROL LAW
        %% =========================
        if ~PF_GOOD
            v = 0.08;
            w = 0;
        else
            [v,w] = feedbackLin(delta(1),delta(2),poseEst(3),epsilon);
        end
    
        [v,w] = limitCmds(v,w,0.2,0.13);
    
        if dist < 0.05
            v = 0; w = 0;
        end
    
        SetFwdVelAngVelCreate(Robot,v,w);
    
        %% =========================
        % PATH ADVANCE
        %% =========================
        if dist < 0.15 && pathIdx < size(currentPath,1)
            pathIdx = pathIdx + 1;
        end
    
        %% =========================
        % DEBUG OUTPUT 
        %% =========================
        fprintf("Goal: [%.2f %.2f] | Dist: %.2f | PathIdx: %d\n", ...
            goal(1), goal(2), norm(poseEst(1:2)' - goal), pathIdx);

        plot(currentPath(:,1), currentPath(:,2), 'b-'); hold on;
        plot(poseEst(1), poseEst(2), 'ro');
        
        fprintf("v=%.2f w=%.2f\n", v, w);
        
        pause(0.05);
        iter = iter + 1;
    end

end