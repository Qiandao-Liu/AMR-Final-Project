function result = finalCompetition(Robot, mapMatPath, opts)
% FINALCOMPETITION Stable final-competition entrypoint based on the
% proven PF waypoint simulator loop.
%
%   result = finalCompetition(Robot)
%   result = finalCompetition(Robot, mapMatPath)
%   result = finalCompetition(Robot, mapMatPath, opts)
%
%   This version intentionally follows the control structure of
%   runPFWaypointTestSimulator and tracks the stored waypoint list in
%   sequence:
%       [mapStruct.waypoints; mapStruct.ECwaypoints]

baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(baseDir, 'src')));

if nargin < 2 || isempty(mapMatPath)
    mapMatPath = fullfile(baseDir, '3credits_practice', 'map1_3credits.mat');
end
if nargin < 3
    opts = struct();
end

mapMatPath = localResolveMapPath(mapMatPath);
mapStruct = load(mapMatPath);
map = mapStruct.map;
candidateStarts = mapStruct.waypoints;

beaconLoc = zeros(0, 3);
if isfield(mapStruct, 'beaconLoc')
    beaconLoc = mapStruct.beaconLoc;
end

if ~isfield(opts, 'waypoints')
    opts.waypoints = localResolveFinalTargets(mapStruct);
end
if ~isfield(opts, 'useGlobalPlanning')
    opts.useGlobalPlanning = true;
end
if ~isfield(opts, 'globalPlannerType')
    opts.globalPlannerType = 'astar';
end
if ~isfield(opts, 'closeEnough')
    opts.closeEnough = 0.20;
end
if ~isfield(opts, 'epsilon')
    opts.epsilon = 0.15;
end
if ~isfield(opts, 'desiredSpeed')
    opts.desiredSpeed = 0.30;
end
if ~isfield(opts, 'maxTime')
    opts.maxTime = 900000;
end
if ~isfield(opts, 'loopPause')
    opts.loopPause = 0.05;
end
if ~isfield(opts, 'wheel2Center')
    opts.wheel2Center = 0.13;
end
if ~isfield(opts, 'maxWheelVelocity')
    opts.maxWheelVelocity = 0.50;
end
if ~isfield(opts, 'sensorOrigin')
    opts.sensorOrigin = [0.13; 0];
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
if ~isfield(opts, 'plotEvery')
    opts.plotEvery = 2;
end
if ~isfield(opts, 'beaconSigma')
    opts.beaconSigma = 0.05;
end
if ~isfield(opts, 'beaconWeightFactor')
    opts.beaconWeightFactor = 6.0;
end
if ~isfield(opts, 'beaconSensorOrigin')
    opts.beaconSensorOrigin = [0; 0];
end
if ~isfield(opts, 'maxAngularSpeed')
    opts.maxAngularSpeed = 1.0;
end
if ~isfield(opts, 'lookaheadDistance')
    opts.lookaheadDistance = 0.25;
end
if ~isfield(opts, 'replanEvery')
    opts.replanEvery = 5;
end
if ~isfield(opts, 'planResolution')
    opts.planResolution = 0.10;
end
if ~isfield(opts, 'robotInflation')
    opts.robotInflation = 0.22;
end
if ~isfield(opts, 'stayAwayInflation')
    opts.stayAwayInflation = 0.25;
end
if ~isfield(opts, 'initTurnRate')
    opts.initTurnRate = 0.5;
end
if ~isfield(opts, 'initMaxTurnAngle')
    opts.initMaxTurnAngle = 2 * pi;
end
if ~isfield(opts, 'initScoreSigma')
    opts.initScoreSigma = 0.20;
end
if ~isfield(opts, 'showWindow')
    opts.showWindow = true;
end

angles = linspace(27 * pi / 180, -27 * pi / 180, 10)';
stayAwayPoints = zeros(0, 2);
if isfield(mapStruct, 'stayAwayPoints')
    stayAwayPoints = mapStruct.stayAwayPoints;
end
boundary = [min(map(:, [1, 3]), [], 'all'), ...
            min(map(:, [2, 4]), [], 'all'), ...
            max(map(:, [1, 3]), [], 'all'), ...
            max(map(:, [2, 4]), [], 'all')];

global dataStore;
dataStore = struct( ...
    'odometry', [], ...
    'rsdepth', [], ...
    'bump', [], ...
    'beacon', [], ...
    'pfEstimate', [], ...
    'targetIdx', [], ...
    'visibleTags', [], ...
    'plannedPath', []);

state = struct();
particlesPre = [];

result = struct();
result.reachedAll = false;
result.finalPoseEstimate = [nan; nan; nan];
result.visitedWaypoints = zeros(0, 2);
result.remainingGoals = opts.waypoints;
result.globalVisitOrder = opts.waypoints;
result.initResult = struct();
result.wallBeliefs = [];
result.replanCount = 0;

iter = 0;
currentPath = zeros(0, 2);
pathTargetIdx = 1;

fig = [];
if opts.showWindow
    fig = figure('Name', 'Final Competition', 'Color', 'w');
end

cleanupObj = onCleanup(@() localCleanup(Robot)); %#ok<NASGU>

SetFwdVelAngVelCreate(Robot, 0, 0);
pause(0.2);

if isempty(opts.waypoints)
    error('finalCompetition:NoWaypoints', ...
        'No waypoint targets available for final competition.');
end

initOpts = struct('turnRate', opts.initTurnRate, ...
                  'loopPause', opts.loopPause, ...
                  'maxTurnAngle', opts.initMaxTurnAngle, ...
                  'headingHypotheses', linspace(-pi, pi, 73), ...
                  'signatureBins', linspace(-pi, pi, 73), ...
                  'wheel2Center', opts.wheel2Center, ...
                  'maxWheelVelocity', opts.maxWheelVelocity, ...
                  'sensorOriginForMatch', [0; 0], ...
                  'scoreSigma', opts.initScoreSigma, ...
                  'verbose', true);
initOpts.headingHypotheses(end) = [];
initOpts.signatureBins(end) = [];

initResult = runInitialLocalizationSimulator(Robot, mapMatPath, initOpts);
result.initResult = initResult;

state.particles = initParticlesFromPose(initResult.bestPose, opts.numParticles);
state.poseEstimate = initResult.bestPose;
result.finalPoseEstimate = initResult.bestPose;

AngleSensorRoomba(Robot);
DistanceSensorRoomba(Robot);

plannedWaypoints = opts.waypoints;
if opts.useGlobalPlanning
    plannedWaypoints = localBuildPlannedOrder( ...
        mapStruct, candidateStarts, opts.waypoints, initResult, opts);
end
opts.waypoints = plannedWaypoints;

targetIdx = localInitialTargetIndex(initResult.bestPose, plannedWaypoints, opts.closeEnough);
if targetIdx > 1
    result.visitedWaypoints = plannedWaypoints(1:targetIdx - 1, :);
end
if targetIdx <= size(plannedWaypoints, 1)
    result.remainingGoals = plannedWaypoints(targetIdx:end, :);
    result.globalVisitOrder = result.remainingGoals;
else
    result.remainingGoals = zeros(0, 2);
    result.globalVisitOrder = zeros(0, 2);
end

if targetIdx > size(plannedWaypoints, 1)
    result.reachedAll = true;
    result.finalPoseEstimate = state.poseEstimate;
    fprintf('Initialization pose is already within tolerance of all final-competition targets.\n');
    return;
end

fprintf('Final competition order:\n');
for i = targetIdx:size(plannedWaypoints, 1)
    fprintf('  %d: [%.2f, %.2f]\n', i, plannedWaypoints(i, 1), plannedWaypoints(i, 2));
end

tic;
while toc < opts.maxTime
    iter = iter + 1;

    dataStore = localReadSensors(Robot, dataStore);

    if isempty(dataStore.odometry) || isempty(dataStore.rsdepth)
        pause(opts.loopPause);
        continue;
    end

    latestOdom = dataStore.odometry(end, 2:3)';
    latestDepth = dataStore.rsdepth(end, 2:end)';
    tags = RealSenseTag(Robot);

    [state, particlesPre] = localizeStepPF( ...
        state, latestOdom, latestDepth, map, opts.sensorOrigin, angles, ...
        struct('processNoise', opts.processNoise, ...
               'measurementNoise', opts.measurementNoise, ...
               'tags', tags, ...
               'beaconLoc', beaconLoc, ...
               'beaconSigma', opts.beaconSigma, ...
               'beaconWeightFactor', opts.beaconWeightFactor, ...
               'beaconSensorOrigin', opts.beaconSensorOrigin));

    poseEst = state.poseEstimate;
    dataStore.pfEstimate = [dataStore.pfEstimate; toc, poseEst'];
    dataStore.targetIdx = [dataStore.targetIdx; toc, targetIdx];
    dataStore.visibleTags = [dataStore.visibleTags; toc, size(tags, 1)];

    currentTarget = plannedWaypoints(targetIdx, :);
    deltaToGoal = currentTarget(:) - poseEst(1:2);
    distToGoal = norm(deltaToGoal);

    if distToGoal <= opts.closeEnough
        fprintf('Reached target %d at [%.3f, %.3f]\n', ...
            targetIdx, currentTarget(1), currentTarget(2));
        if isempty(result.visitedWaypoints)
            result.visitedWaypoints = currentTarget;
        else
            result.visitedWaypoints = [result.visitedWaypoints; currentTarget];
        end
        targetIdx = targetIdx + 1;

        if targetIdx > size(plannedWaypoints, 1)
            result.reachedAll = true;
            result.remainingGoals = zeros(0, 2);
            result.globalVisitOrder = zeros(0, 2);
            break;
        end

        currentPath = zeros(0, 2);
        pathTargetIdx = 1;
        result.remainingGoals = plannedWaypoints(targetIdx:end, :);
        result.globalVisitOrder = result.remainingGoals;
        currentTarget = plannedWaypoints(targetIdx, :);
    end

    plannerOpts = struct('resolution', opts.planResolution, ...
                         'inflationRadius', opts.robotInflation, ...
                         'stayAwayPoints', stayAwayPoints, ...
                         'stayAwayRadius', opts.stayAwayInflation);

    if isempty(currentPath) || mod(iter, opts.replanEvery) == 1
        previousPath = currentPath;
        [candidatePath, ~, ~, found] = planPathAStar(map, boundary, poseEst(1:2)', currentTarget, plannerOpts);
        if found && ~isempty(candidatePath)
            currentPath = candidatePath;
            dataStore.plannedPath = currentPath;
            pathTargetIdx = 1;
        elseif isempty(previousPath)
            warning('A* failed to find an initial path from PF estimate to target %d. Stopping.', targetIdx);
            break;
        else
            warning('A* replanning failed at target %d. Continuing on previous path.', targetIdx);
        end
    end

    while pathTargetIdx < size(currentPath, 1) && ...
            norm(currentPath(pathTargetIdx, :)' - poseEst(1:2)) < opts.lookaheadDistance
        pathTargetIdx = pathTargetIdx + 1;
    end

    localTarget = currentPath(pathTargetIdx, :);
    delta = localTarget(:) - poseEst(1:2);
    distToLocal = norm(delta);

    if distToLocal < 1e-6
        desiredVel = [0; 0];
    else
        speed = opts.desiredSpeed;
        if distToLocal < 0.4
            speed = speed * max(distToLocal / 0.4, 0.25);
        end
        desiredVel = speed * delta / distToLocal;
    end

    [cmdV, cmdW] = feedbackLin(desiredVel(1), desiredVel(2), poseEst(3), opts.epsilon);
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, opts.maxWheelVelocity, opts.wheel2Center);
    cmdW = max(min(cmdW, opts.maxAngularSpeed), -opts.maxAngularSpeed);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

    if mod(iter, opts.plotEvery) == 0 && ~isempty(fig) && isvalid(fig)
        localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, opts.waypoints, targetIdx, ...
            currentPath, pathTargetIdx, particlesPre, dataStore);
    end

    if mod(iter, 10) == 0
        fprintf(['PF estimate: [x=%.3f, y=%.3f, th=%.3f], target=%d, ', ...
                 'goalDist=%.3f, pathIdx=%d, tags=%d\n'], ...
            poseEst(1), poseEst(2), poseEst(3), targetIdx, distToGoal, pathTargetIdx, size(tags, 1));
    end

    pause(opts.loopPause);
end

SetFwdVelAngVelCreate(Robot, 0, 0);

if ~isempty(fig) && isvalid(fig)
    localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, plannedWaypoints, ...
        min(targetIdx, size(plannedWaypoints, 1)), currentPath, pathTargetIdx, particlesPre, dataStore);
end

result.finalPoseEstimate = state.poseEstimate;

if result.reachedAll
    fprintf('Completed all %d final-competition targets.\n', size(plannedWaypoints, 1));
else
    fprintf('Stopped before completing all final-competition targets.\n');
end
fprintf('Final PF estimate: [x=%.3f, y=%.3f, th=%.3f]\n', ...
    result.finalPoseEstimate(1), result.finalPoseEstimate(2), result.finalPoseEstimate(3));
end

function plannedWaypoints = localBuildPlannedOrder(mapStruct, candidateStarts, goalPoints, initResult, opts)
plannedWaypoints = goalPoints;

if isempty(goalPoints)
    return;
end

stayAwayPoints = zeros(0, 2);
if isfield(mapStruct, 'stayAwayPoints')
    stayAwayPoints = mapStruct.stayAwayPoints;
end

boundary = [min(mapStruct.map(:, [1, 3]), [], 'all'), ...
            min(mapStruct.map(:, [2, 4]), [], 'all'), ...
            max(mapStruct.map(:, [1, 3]), [], 'all'), ...
            max(mapStruct.map(:, [2, 4]), [], 'all')];

plannerOpts = struct( ...
    'plannerType', opts.globalPlannerType, ...
    'boundary', boundary, ...
    'stayAwayPoints', stayAwayPoints, ...
    'planResolution', opts.planResolution, ...
    'robotInflation', opts.robotInflation, ...
    'stayAwayInflation', opts.stayAwayInflation, ...
    'nPRM', 150);

    [~, costMatrix, ~, nodeMeta] = precomputePairwisePathCosts( ...
        mapStruct.map, candidateStarts, goalPoints, plannerOpts);

startNodeIdx = nodeMeta.candidateStartNodeIdx(initResult.bestWaypointIdx);
goalIdx = 1:size(goalPoints, 1);
startXY = candidateStarts(initResult.bestWaypointIdx, :);
visitedStartMask = all(goalPoints == startXY, 2) & ...
    vecnorm(goalPoints - initResult.bestPose(1:2)', 2, 2) <= opts.closeEnough;
remainingGoalIdx = goalIdx(~visitedStartMask);

if isempty(remainingGoalIdx)
    plannedWaypoints = zeros(0, 2);
    return;
end

remainingGoalNodeIdx = nodeMeta.goalNodeIdx(remainingGoalIdx);
[~, visitGoalOrder, totalCost] = solveGlobalVisitOrder(costMatrix, startNodeIdx, remainingGoalNodeIdx);
if isempty(visitGoalOrder)
    warning('finalCompetition:GlobalPlanningFailed', ...
        'Global planning failed; falling back to stored target order.');
    return;
end

plannedWaypoints = goalPoints(remainingGoalIdx(visitGoalOrder), :);
fprintf('Global planning cost: %.3f over %d remaining goals.\n', totalCost, size(plannedWaypoints, 1));
end

function waypoints = localResolveFinalTargets(mapStruct)
waypoints = zeros(0, 2);

if isfield(mapStruct, 'waypoints') && ~isempty(mapStruct.waypoints)
    waypoints = mapStruct.waypoints;
end
if isfield(mapStruct, 'ECwaypoints') && ~isempty(mapStruct.ECwaypoints)
    waypoints = [waypoints; mapStruct.ECwaypoints];
end
end

function targetIdx = localInitialTargetIndex(startPose, waypoints, closeEnough)
targetIdx = 1;

if isempty(waypoints)
    return;
end

startPos = startPose(1:2)';
while targetIdx <= size(waypoints, 1) && norm(waypoints(targetIdx, :) - startPos) <= closeEnough
    targetIdx = targetIdx + 1;
end
end

function localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, waypoints, targetIdx, currentPath, pathTargetIdx, particlesPre, dataStore)
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
    for i = 1:size(beaconLoc, 1)
        text(ax, beaconLoc(i, 2) + 0.08, beaconLoc(i, 3) + 0.08, ...
            sprintf('%d', beaconLoc(i, 1)), 'Color', [0.5, 0, 0.5], 'FontSize', 8);
    end
end

if ~isempty(stayAwayPoints)
    plot(ax, stayAwayPoints(:, 1), stayAwayPoints(:, 2), 'x', ...
        'Color', [1, 0.5, 0], 'MarkerSize', 8, 'LineWidth', 1.5);
end

plot(ax, waypoints(:, 1), waypoints(:, 2), 'bo', 'MarkerFaceColor', 'c', 'MarkerSize', 8);
if ~isempty(waypoints) && targetIdx <= size(waypoints, 1)
    plot(ax, waypoints(targetIdx, 1), waypoints(targetIdx, 2), 'rp', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 14);
end

if ~isempty(currentPath)
    plot(ax, currentPath(:, 1), currentPath(:, 2), 'b-', 'LineWidth', 1.2);
    plot(ax, currentPath(pathTargetIdx, 1), currentPath(pathTargetIdx, 2), 'ko', ...
        'MarkerFaceColor', 'y', 'MarkerSize', 8);
end

if ~isempty(particlesPre)
    scatter(ax, particlesPre.poses(1, :), particlesPre.poses(2, :), ...
        10, particlesPre.weights, 'filled', 'MarkerFaceAlpha', 0.35);
end

if ~isempty(dataStore.pfEstimate)
    plot(ax, dataStore.pfEstimate(:, 2), dataStore.pfEstimate(:, 3), 'r-', 'LineWidth', 1.5);
    plot(ax, dataStore.pfEstimate(end, 2), dataStore.pfEstimate(end, 3), 'ro', ...
        'MarkerFaceColor', 'r');
end

xlim(ax, [min(map(:, [1, 3]), [], 'all') - 0.5, max(map(:, [1, 3]), [], 'all') + 0.5]);
ylim(ax, [min(map(:, [2, 4]), [], 'all') - 0.5, max(map(:, [2, 4]), [], 'all') + 0.5]);
numVisible = 0;
if ~isempty(dataStore.visibleTags)
    numVisible = dataStore.visibleTags(end, 2);
end
title(ax, sprintf('Final Competition PF + A* (visible tags: %d)', numVisible));
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
drawnow limitrate;
end

function localCleanup(Robot)
try
    SetFwdVelAngVelCreate(Robot, 0, 0);
catch
end
end

function dataStore = localReadSensors(Robot, dataStore)
try
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

try
    deltaD = DistanceSensorRoomba(CreatePort);
    deltaA = AngleSensorRoomba(CreatePort);
    dataStore.odometry = [dataStore.odometry; toc, deltaD, deltaA];
catch
    disp('Error retrieving or saving odometry data.');
end

try
    [BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront] = ...
        BumpsWheelDropsSensorsRoomba(CreatePort);
    dataStore.bump = [dataStore.bump; toc, ...
        BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront];
catch
    disp('Error retrieving or saving bump sensor data.');
end

try
    depthArray = RealSenseDist(Robot);
    dataStore.rsdepth = [dataStore.rsdepth; toc, depthArray'];
catch
    disp('Error retrieving or saving RealSense depth data.');
end

try
    tags = RealSenseTag(Robot);
    if ~isempty(tags)
        dataStore.beacon = [dataStore.beacon; repmat(toc, size(tags, 1), 1), tags];
    end
catch
    disp('Error retrieving or saving beacon data.');
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

error('finalCompetition:MapNotFound', ...
    'Unable to find map file ''%s'' or a matching .mat file.', mapMatPath);
end
