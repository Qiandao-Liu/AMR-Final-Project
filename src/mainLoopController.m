function result = mainLoopController(Robot, mapMatPath, opts)

    if nargin < 2 || isempty(mapMatPath)
        baseDir = fileparts(fileparts(mfilename('fullpath')));
        mapMatPath = fullfile(baseDir, '3credits_practice', 'map1_3credits.mat');
    end
    if nargin < 3
        opts = struct();
    end

    mapMatPath = localResolveMapPath(mapMatPath);
    opts = localApplyDefaults(opts);

    result = struct('finalPoseEstimate', [nan; nan; nan], ...
                    'visitedWaypoints', zeros(0, 2), ...
                    'remainingWaypoints', zeros(0, 2));

    %% =========================
    % LOAD MAP
    % =========================
    mapStruct = load(mapMatPath);

    map = mapStruct.map;
    waypoints = mapStruct.waypoints;

    ECwaypoints = [];
    if isfield(mapStruct, 'ECwaypoints')
        ECwaypoints = mapStruct.ECwaypoints;
    end

    stayAwayPoints = [];
    if isfield(mapStruct, 'stayAwayPoints')
        stayAwayPoints = mapStruct.stayAwayPoints;
    end

    beaconLoc = [];
    if isfield(mapStruct, 'beaconLoc')
        beaconLoc = mapStruct.beaconLoc;
    end

    allWaypoints = [waypoints; ECwaypoints];

    simBound = [ ...
        min(map(:, [1, 3]), [], 'all'), ...
        min(map(:, [2, 4]), [], 'all'), ...
        max(map(:, [1, 3]), [], 'all'), ...
        max(map(:, [2, 4]), [], 'all')];

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
        'visitedWaypoints', [], ...
        'visibleTags', [], ...
        'plannedPath', []);

    %% =========================
    % INITIAL LOCALIZATION
    % =========================
    initOpts = struct('turnRate', 0.5, ...
                      'loopPause', opts.loopPause, ...
                      'maxTurnAngle', 2 * pi, ...
                      'verbose', true);

    initResult = runInitialLocalizationSimulator(Robot, mapMatPath, initOpts);

    %% =========================
    % INITIALIZE PF
    % =========================
    state.particles = initParticlesFromPose(initResult.bestPose, opts.numParticles);
    state.poseEstimate = initResult.bestPose;
    result.finalPoseEstimate = initResult.bestPose;

    AngleSensorRoomba(Robot);
    DistanceSensorRoomba(Robot);

    fig = [];
    if opts.showPFWindow
        fig = figure('Name', 'Main Loop PF View', 'Color', 'w');
    end

    %% =========================
    % BUILD PRM
    % =========================
    robotRadius = 0.2;
    nPRM = 150;

    obstacles = buildObstacles(stayAwayPoints);

    PRM = buildPRM(obstacles, map, simBound, ...
                   state.poseEstimate(1:2)', allWaypoints(1, :), ...
                   nPRM, robotRadius, stayAwayPoints, @haltonSample);

    %% =========================
    % PARAMETERS
    % =========================
    closeEnough = 0.2;
    epsilon = 0.15;
    maxTime = 300;
    lookaheadDist = 0.3;

    remainingWaypoints = allWaypoints;
    currentPath = [];
    currentGoal = [];
    pathIdx = 1;
    poseEstFiltered = state.poseEstimate;
    particlesPre = [];

    tic;
    iter = 1;
    while toc < maxTime

        %% =========================
        % SENSOR UPDATE
        %% =========================
        dataStore = readAllSensors(Robot, dataStore);

        if isempty(dataStore.odometry) || isempty(dataStore.rsdepth)
            pause(opts.loopPause);
            continue;
        end

        latestOdom = dataStore.odometry(end, 2:3)';
        latestDepth = dataStore.rsdepth(end, 2:end)';
        tags = RealSenseTag(Robot);

        %% =========================
        % PARTICLE FILTER UPDATE
        %% =========================
        [state, particlesPre] = localizeStepPF( ...
            state, latestOdom, latestDepth, map, ...
            [0.13; 0], linspace(27 * pi / 180, -27 * pi / 180, 10)', ...
            struct('processNoise', opts.processNoise, ...
                   'measurementNoise', opts.measurementNoise, ...
                   'tags', tags));

        %% =========================
        % PF HEALTH METRICS
        %% =========================
        w = particlesPre.weights;
        p = particlesPre.poses;

        pfSpread = sqrt(var(p(1, :)) + var(p(2, :)));
        neff = 1 / sum(w .^ 2);
        pfGood = (pfSpread > 0.03) && (neff > 30);

        fprintf('PF spread: %.3f | Neff: %.1f\n', pfSpread, neff);

        %% =========================
        % STABLE POSE ESTIMATE
        %% =========================
        poseEst = state.poseEstimate;
        if ~pfGood
            poseEst = poseEstFiltered;
        else
            poseEstFiltered = 0.8 * poseEstFiltered + 0.2 * poseEst;
            poseEst = poseEstFiltered;
        end

        %% =========================
        % LOGGING
        %% =========================
        dataStore.pfPose = [dataStore.pfPose; toc, poseEst'];
        dataStore.visibleTags = [dataStore.visibleTags; toc, size(tags, 1)];

        %% =========================
        % WAYPOINT PROGRESSION
        %% =========================
        if isempty(remainingWaypoints)
            break;
        end

        dists = vecnorm(remainingWaypoints - poseEst(1:2)', 2, 2);
        [minDist, idxMin] = min(dists);

        if minDist < closeEnough
            goal = remainingWaypoints(idxMin, :);

            SetFwdVelAngVelCreate(Robot, 0, 0);
            SetLEDsRoomba(Robot, 3, 100, 100);
            pause(1);

            dataStore.visitedWaypoints = [dataStore.visitedWaypoints; goal];
            remainingWaypoints(idxMin, :) = [];

            result.visitedWaypoints = dataStore.visitedWaypoints;
            result.remainingWaypoints = remainingWaypoints;

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
            dataStore.plannedPath = currentPath;
            pathIdx = 2;

            if isempty(currentPath)
                disp('PRM failed -> retrying next cycle');
                pause(opts.loopPause);
                continue;
            end

            SetLEDsRoomba(Robot, 3, 0, 100);
        end

        %% =========================
        % PATH FOLLOWING
        %% =========================
        while pathIdx < size(currentPath, 1) && ...
              norm(currentPath(pathIdx, :) - poseEst(1:2)') < lookaheadDist
            pathIdx = pathIdx + 1;
        end

        target = currentPath(pathIdx, :);
        delta = target - poseEst(1:2)';
        dist = norm(delta);

        %% =========================
        % CONTROL LAW
        %% =========================
        if ~pfGood
            v = 0.08;
            wCmd = 0;
        else
            [v, wCmd] = feedbackLin(delta(1), delta(2), poseEst(3), epsilon);
        end

        [v, wCmd] = limitCmds(v, wCmd, 0.2, 0.13);

        if dist < 0.05
            v = 0;
            wCmd = 0;
        end

        SetFwdVelAngVelCreate(Robot, v, wCmd);

        %% =========================
        % PATH ADVANCE
        %% =========================
        if dist < 0.15 && pathIdx < size(currentPath, 1)
            pathIdx = pathIdx + 1;
        end

        %% =========================
        % DEBUG OUTPUT
        %% =========================
        fprintf('Goal: [%.2f %.2f] | Dist: %.2f | PathIdx: %d\n', ...
            goal(1), goal(2), norm(poseEst(1:2)' - goal), pathIdx);
        fprintf('v=%.2f w=%.2f\n', v, wCmd);

        result.finalPoseEstimate = poseEst;
        result.remainingWaypoints = remainingWaypoints;

        if opts.showPFWindow && ~isempty(fig) && isvalid(fig) && mod(iter, opts.plotEvery) == 0
            localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, allWaypoints, ...
                remainingWaypoints, currentGoal, currentPath, pathIdx, particlesPre, dataStore);
        end

        pause(opts.loopPause);
        iter = iter + 1;
    end

    if opts.showPFWindow && ~isempty(fig) && isvalid(fig)
        localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, allWaypoints, ...
            remainingWaypoints, currentGoal, currentPath, pathIdx, particlesPre, dataStore);
    end

end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'plotEvery')
    opts.plotEvery = 2;
end
if ~isfield(opts, 'showPFWindow')
    opts.showPFWindow = true;
end
if ~isfield(opts, 'loopPause')
    opts.loopPause = 0.05;
end
if ~isfield(opts, 'numParticles')
    opts.numParticles = 500;
end
if ~isfield(opts, 'processNoise')
    opts.processNoise = diag([0.004, 0.004, 0.003]);
end
if ~isfield(opts, 'measurementNoise')
    opts.measurementNoise = 0.08 * eye(10);
end
end

function mapMatPath = localResolveMapPath(mapMatPath)
mapMatPath = string(mapMatPath);

if endsWith(lower(mapMatPath), ".txt")
    txtPath = mapMatPath;
    matPath = replace(txtPath, ".txt", ".mat");

    if isfile(matPath)
        mapMatPath = matPath;
        return;
    end
end

if isfile(mapMatPath)
    return;
end

[~, mapName, ext] = fileparts(mapMatPath);
baseDir = fileparts(fileparts(mfilename('fullpath')));

candidates = [
    fullfile(baseDir, '3credits_practice', mapName + ".mat")
    fullfile(baseDir, '3credits_practice', mapName + ext)
];

for i = 1:numel(candidates)
    if isfile(candidates(i))
        mapMatPath = candidates(i);
        return;
    end
end

error('mainLoopController:MapNotFound', ...
    'Unable to find map file ''%s'' or a matching .mat file.', mapMatPath);
end

function localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, allWaypoints, remainingWaypoints, currentGoal, currentPath, pathIdx, particlesPre, dataStore)
figure(fig);
clf(fig);
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');

for i = 1:size(map, 1)
    plot(ax, [map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'k-', 'LineWidth', 2);
end

if ~isempty(beaconLoc)
    plot(ax, beaconLoc(:, 2), beaconLoc(:, 3), 'ms', ...
        'MarkerFaceColor', 'm', 'MarkerSize', 7);
end

if ~isempty(stayAwayPoints)
    plot(ax, stayAwayPoints(:, 1), stayAwayPoints(:, 2), 'x', ...
        'Color', [1, 0.5, 0], 'MarkerSize', 8, 'LineWidth', 1.5);
end

if ~isempty(allWaypoints)
    plot(ax, allWaypoints(:, 1), allWaypoints(:, 2), 'bo', ...
        'MarkerFaceColor', 'c', 'MarkerSize', 8);
end

if ~isempty(dataStore.visitedWaypoints)
    plot(ax, dataStore.visitedWaypoints(:, 1), dataStore.visitedWaypoints(:, 2), 'go', ...
        'MarkerFaceColor', 'g', 'MarkerSize', 8);
end

if ~isempty(remainingWaypoints)
    plot(ax, remainingWaypoints(:, 1), remainingWaypoints(:, 2), 'ro', ...
        'MarkerFaceColor', 'y', 'MarkerSize', 9);
end

if ~isempty(currentGoal)
    plot(ax, currentGoal(1), currentGoal(2), 'rp', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 14);
end

if ~isempty(currentPath)
    plot(ax, currentPath(:, 1), currentPath(:, 2), 'b-', 'LineWidth', 1.2);
    pathIdx = min(max(pathIdx, 1), size(currentPath, 1));
    plot(ax, currentPath(pathIdx, 1), currentPath(pathIdx, 2), 'ko', ...
        'MarkerFaceColor', 'y', 'MarkerSize', 8);
end

if ~isempty(particlesPre)
    scatter(ax, particlesPre.poses(1, :), particlesPre.poses(2, :), ...
        10, particlesPre.weights, 'filled', 'MarkerFaceAlpha', 0.35);
end

if ~isempty(dataStore.pfPose)
    plot(ax, dataStore.pfPose(:, 2), dataStore.pfPose(:, 3), 'r-', 'LineWidth', 1.5);
    plot(ax, dataStore.pfPose(end, 2), dataStore.pfPose(end, 3), 'ro', ...
        'MarkerFaceColor', 'r');
end

xlim(ax, [min(map(:, [1, 3]), [], 'all') - 0.5, max(map(:, [1, 3]), [], 'all') + 0.5]);
ylim(ax, [min(map(:, [2, 4]), [], 'all') - 0.5, max(map(:, [2, 4]), [], 'all') + 0.5]);

numVisible = 0;
if ~isempty(dataStore.visibleTags)
    numVisible = dataStore.visibleTags(end, 2);
end

title(ax, sprintf('Main Loop Controller PF View (visible tags: %d)', numVisible));
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
drawnow limitrate;
end
