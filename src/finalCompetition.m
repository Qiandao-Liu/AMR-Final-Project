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

if nargin < 3
    opts = struct();
end

if nargin < 2 || isempty(mapMatPath)
    mapMatPath = localResolveMapDirectoryPath(baseDir);
else
    mapMatPath = localResolveMapPath(mapMatPath);
end
fprintf('Using map file: %s\n', char(mapMatPath));
mapStruct = load(char(mapMatPath));
map = mapStruct.map;
candidateStarts = mapStruct.waypoints;
optWalls = localOptionalField(mapStruct, 'optWalls', zeros(0, 4));

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
    opts.desiredSpeed = 0.12;
end
if ~isfield(opts, 'maxForwardSpeed')
    opts.maxForwardSpeed = 0.14;
end
if ~isfield(opts, 'maxReverseSpeed')
    opts.maxReverseSpeed = 0.04;
end
if ~isfield(opts, 'maxTime')
    opts.maxTime = 300;
end
if ~isfield(opts, 'loopPause')
    opts.loopPause = 0.05;
end
if ~isfield(opts, 'wheel2Center')
    opts.wheel2Center = 0.13;
end
if ~isfield(opts, 'maxWheelVelocity')
    opts.maxWheelVelocity = 0.25;
end
if ~isfield(opts, 'sensorOrigin')
    opts.sensorOrigin = [0.08; 0];
end
if ~isfield(opts, 'numParticles')
    opts.numParticles = 500;
end
if ~isfield(opts, 'processNoise')
    opts.processNoise = diag([0.004, 0.004, 0.003]);
end
if ~isfield(opts, 'useScaledProcessNoise')
    opts.useScaledProcessNoise = true;
end
if ~isfield(opts, 'processNoisePositionBase')
    opts.processNoisePositionBase = 0.006;
end
if ~isfield(opts, 'processNoisePositionPerMeter')
    opts.processNoisePositionPerMeter = 0.08;
end
if ~isfield(opts, 'processNoiseThetaBase')
    opts.processNoiseThetaBase = 0.004;
end
if ~isfield(opts, 'processNoiseThetaPerRad')
    opts.processNoiseThetaPerRad = 0.08;
end
if ~isfield(opts, 'measurementNoise')
    opts.measurementNoise = 0.08 * eye(10);
end
if ~isfield(opts, 'plotEvery')
    opts.plotEvery = 2;
end
if ~isfield(opts, 'beaconSigma')
    opts.beaconSigma = 0.04;
end
if ~isfield(opts, 'beaconWeightFactor')
    opts.beaconWeightFactor = 8.0;
end
if ~isfield(opts, 'beaconSensorOrigin')
    opts.beaconSensorOrigin = [0; 0];
end
if ~isfield(opts, 'maxAngularSpeed')
    opts.maxAngularSpeed = 0.6;
end
if ~isfield(opts, 'lookaheadDistance')
    opts.lookaheadDistance = 0.25;
end
if ~isfield(opts, 'nearGoalLookaheadDistance')
    opts.nearGoalLookaheadDistance = 0.15;
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
if ~isfield(opts, 'preferredClearance')
    opts.preferredClearance = opts.robotInflation + 0.25;
end
if ~isfield(opts, 'clearanceWeight')
    opts.clearanceWeight = 2.0;
end
if ~isfield(opts, 'maxShortcutLength')
    opts.maxShortcutLength = 0.25;
end
if ~isfield(opts, 'nearGoalShortcutLength')
    opts.nearGoalShortcutLength = 0.12;
end
if ~isfield(opts, 'nearGoalSlowRadius')
    opts.nearGoalSlowRadius = 0.80;
end
if ~isfield(opts, 'nearGoalMinSpeed')
    opts.nearGoalMinSpeed = 0.055;
end
if ~isfield(opts, 'snakeAngularThreshold')
    opts.snakeAngularThreshold = 0.45;
end
if ~isfield(opts, 'snakeProgressThreshold')
    opts.snakeProgressThreshold = 0.003;
end
if ~isfield(opts, 'snakeTriggerCount')
    opts.snakeTriggerCount = 18;
end
if ~isfield(opts, 'snakeRecoverySteps')
    opts.snakeRecoverySteps = 20;
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
if ~isfield(opts, 'controlPoseAlpha')
    opts.controlPoseAlpha = 0.20;
end
if ~isfield(opts, 'controlPoseAlphaWithTags')
    opts.controlPoseAlphaWithTags = 0.35;
end
if ~isfield(opts, 'maxPoseStepForControl')
    opts.maxPoseStepForControl = 0.18;
end
if ~isfield(opts, 'maxThetaStepForControl')
    opts.maxThetaStepForControl = 0.45;
end
if ~isfield(opts, 'pfMaxSpreadBeforeReseed')
    opts.pfMaxSpreadBeforeReseed = 0.45;
end
if ~isfield(opts, 'pfMinNeffBeforeReseed')
    opts.pfMinNeffBeforeReseed = 20;
end
if ~isfield(opts, 'pfReseedOnTags')
    opts.pfReseedOnTags = false;
end
if ~isfield(opts, 'pfReseedPositionSigma')
    opts.pfReseedPositionSigma = 0.12;
end
if ~isfield(opts, 'pfReseedThetaSigma')
    opts.pfReseedThetaSigma = 15 * pi / 180;
end
if ~isfield(opts, 'tagSnapAlpha')
    opts.tagSnapAlpha = 0.65;
end
if ~isfield(opts, 'ledPause')
    opts.ledPause = 0.2;
end
if ~isfield(opts, 'activeNavMapMode')
    opts.activeNavMapMode = 'confirmed';
end
if ~isfield(opts, 'activeSenseReliableRange')
    opts.activeSenseReliableRange = 1.0;
end
if ~isfield(opts, 'activeSenseSafetyEnabled')
    opts.activeSenseSafetyEnabled = false;
end
if ~isfield(opts, 'activeSenseStopRange')
    opts.activeSenseStopRange = 0.34;
end
if ~isfield(opts, 'activeSenseForwardFov')
    opts.activeSenseForwardFov = 12 * pi / 180;
end
if ~isfield(opts, 'activeSensePathCorridor')
    opts.activeSensePathCorridor = 0.22;
end
if ~isfield(opts, 'activeSensePathLookahead')
    opts.activeSensePathLookahead = 0.55;
end
if ~isfield(opts, 'activeSenseUnexpectedMargin')
    opts.activeSenseUnexpectedMargin = 0.18;
end
if ~isfield(opts, 'activeSenseReplanCooldown')
    opts.activeSenseReplanCooldown = 5;
end

angles = linspace(27 * pi / 180, -27 * pi / 180, 10)';
stayAwayPoints = localOptionalField(mapStruct, 'StayAwayPoints', ...
    localOptionalField(mapStruct, 'stayAwayPoints', zeros(0, 2)));
boundary = [min(map(:, [1, 3]), [], 'all'), ...
            min(map(:, [2, 4]), [], 'all'), ...
            max(map(:, [1, 3]), [], 'all'), ...
            max(map(:, [2, 4]), [], 'all')];
wallBeliefs = initWallBeliefs(optWalls);
[navMap, activePlotData] = buildActiveNavMap(map, optWalls, wallBeliefs, opts.activeNavMapMode);

global dataStore;
dataStore = struct( ...
    'odometry', [], ...
    'rsdepth', [], ...
    'bump', [], ...
    'beacon', [], ...
    'robotPose', [], ...
    'pfEstimate', [], ...
    'targetIdx', [], ...
    'visibleTags', [], ...
    'plannedPath', [], ...
    'visitedWaypoints', zeros(0, 2), ...
    'wallBeliefs', wallBeliefs, ...
    'activeNavMap', navMap, ...
    'activeSenseEvents', []);

state = struct();
particlesPre = [];

result = struct();
result.reachedAll = false;
result.finalPoseEstimate = [nan; nan; nan];
result.visitedWaypoints = zeros(0, 2);
result.remainingGoals = opts.waypoints;
result.globalVisitOrder = opts.waypoints;
result.initResult = struct();
result.wallBeliefs = wallBeliefs;
result.activePlotData = activePlotData;
result.replanCount = 0;
result.mapMatPath = mapMatPath;
result.outputDataPath = "";
result.outputPlotPath = "";
result.outputFigurePath = "";
result.stopReason = "not_started";
result.stopDetails = "";

iter = 0;
currentPath = zeros(0, 2);
pathTargetIdx = 1;
snakeCount = 0;
snakeRecoveryCounter = 0;
prevDistToGoal = inf;
prevCmdWSign = 0;

fig = [];
if opts.showWindow
    fig = figure('Name', 'Final Competition', 'Color', 'w');
end

cleanupObj = onCleanup(@() localCleanup(Robot));

SetFwdVelAngVelCreate(Robot, 0, 0);
localSetPowerLED(Robot, 'green', opts);
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
controlPose = initResult.bestPose;

AngleSensorRoomba(Robot);
DistanceSensorRoomba(Robot);

plannedWaypoints = opts.waypoints;
if opts.useGlobalPlanning
    activeMapStruct = mapStruct;
    activeMapStruct.map = navMap;
    activeMapStruct.knownMap = map;
    plannedWaypoints = localBuildPlannedOrder( ...
        activeMapStruct, candidateStarts, opts.waypoints, initResult, opts);
end
opts.waypoints = plannedWaypoints;
result.globalVisitOrder = plannedWaypoints;

targetIdx = localInitialTargetIndex(initResult.bestPose, plannedWaypoints, opts.closeEnough);
if targetIdx > 1
    result.visitedWaypoints = plannedWaypoints(1:targetIdx - 1, :);
    dataStore.visitedWaypoints = result.visitedWaypoints;
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
    result.stopReason = "completed";
    result.stopDetails = "Initialization pose is already within tolerance of all final-competition targets.";
    result.finalPoseEstimate = state.poseEstimate;
    result.wallBeliefs = wallBeliefs;
    result.activePlotData = activePlotData;
    dataStore.visitedWaypoints = result.visitedWaypoints;
    dataStore.wallBeliefs = wallBeliefs;
    dataStore.activeNavMap = navMap;
    [result.outputDataPath, result.outputPlotPath, result.outputFigurePath] = ...
        localSaveCompetitionOutput(baseDir, result, dataStore, mapStruct, plannedWaypoints);
    fprintf('Initialization pose is already within tolerance of all final-competition targets.\n');
    return;
end

fprintf('Final competition order:\n');
for i = targetIdx:size(plannedWaypoints, 1)
    fprintf('  %d: [%.2f, %.2f]\n', i, plannedWaypoints(i, 1), plannedWaypoints(i, 2));
end

activeReplanCooldown = 0;
stopReason = "time_limit";
stopDetails = sprintf('Time limit reached after %.1f seconds.', opts.maxTime);
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
    processNoise = opts.processNoise;
    if opts.useScaledProcessNoise
        processNoise = localScaledProcessNoise(latestOdom, opts);
    end

    [state, particlesPre] = localizeStepPF( ...
        state, latestOdom, latestDepth, map, opts.sensorOrigin, angles, ...
        struct('processNoise', processNoise, ...
               'measurementNoise', opts.measurementNoise, ...
               'tags', tags, ...
               'beaconLoc', beaconLoc, ...
               'beaconSigma', opts.beaconSigma, ...
               'beaconWeightFactor', opts.beaconWeightFactor, ...
               'beaconSensorOrigin', opts.beaconSensorOrigin));

    [pfSpread, neff] = localPFHealth(particlesPre);
    poseEst = state.poseEstimate;
    if opts.pfReseedOnTags && localShouldReseedPF(pfSpread, neff, size(tags, 1), opts)
        fprintf('PF reseed near control pose | spread=%.3f Neff=%.1f tags=%d\n', ...
            pfSpread, neff, size(tags, 1));
        state.particles = initParticlesFromPose( ...
            controlPose, opts.numParticles, opts.pfReseedPositionSigma, opts.pfReseedThetaSigma);
        state.poseEstimate = controlPose;
        poseEst = controlPose;
        particlesPre = [];
    end
    controlPose = localUpdateControlPose(controlPose, poseEst, size(tags, 1), opts);
    poseCtrl = controlPose;
    dataStore.pfEstimate = [dataStore.pfEstimate; toc, poseCtrl'];
    dataStore.robotPose = dataStore.pfEstimate;
    dataStore.targetIdx = [dataStore.targetIdx; toc, targetIdx];
    dataStore.visibleTags = [dataStore.visibleTags; toc, size(tags, 1)];
    if activeReplanCooldown > 0
        activeReplanCooldown = activeReplanCooldown - 1;
    end

    beliefOpts = struct( ...
        'maxReliableRange', opts.activeSenseReliableRange);
    [wallBeliefs, ~, wallPresentChanged, wallAbsentChanged] = updateWallBeliefs( ...
        poseCtrl, latestDepth, map, optWalls, wallBeliefs, opts.sensorOrigin, angles, beliefOpts);
    [navMap, activePlotData] = buildActiveNavMap(map, optWalls, wallBeliefs, opts.activeNavMapMode);
    result.wallBeliefs = wallBeliefs;
    result.activePlotData = activePlotData;
    dataStore.wallBeliefs = wallBeliefs;
    dataStore.activeNavMap = navMap;

    if wallAbsentChanged
        fprintf('Active sensing marked an optional wall absent; no replan needed.\n');
    end

    if wallPresentChanged
        SetFwdVelAngVelCreate(Robot, 0, 0);
        pause(opts.loopPause);
        fprintf('Active sensing confirmed optional wall present; replanning remaining goals.\n');
        remainingGoals = plannedWaypoints(targetIdx:end, :);
        plannedWaypoints = localReorderRemainingWaypoints( ...
            navMap, boundary, poseCtrl, result.visitedWaypoints, remainingGoals, stayAwayPoints, opts);
        opts.waypoints = plannedWaypoints;
        targetIdx = size(result.visitedWaypoints, 1) + 1;
        result.remainingGoals = plannedWaypoints(targetIdx:end, :);
        result.globalVisitOrder = result.remainingGoals;
        currentPath = zeros(0, 2);
        pathTargetIdx = 1;
        result.replanCount = result.replanCount + 1;
        activeReplanCooldown = opts.activeSenseReplanCooldown;
        continue;
    end

    currentTarget = plannedWaypoints(targetIdx, :);
    deltaToGoal = currentTarget(:) - poseCtrl(1:2);
    distToGoal = norm(deltaToGoal);

    if distToGoal <= opts.closeEnough
        SetFwdVelAngVelCreate(Robot, 0, 0);
        localSetPowerLED(Robot, 'red', opts);
        fprintf('Reached target %d at [%.3f, %.3f]\n', ...
            targetIdx, currentTarget(1), currentTarget(2));
        if isempty(result.visitedWaypoints)
            result.visitedWaypoints = currentTarget;
        else
            result.visitedWaypoints = [result.visitedWaypoints; currentTarget];
        end
        dataStore.visitedWaypoints = result.visitedWaypoints;
        targetIdx = targetIdx + 1;

        if targetIdx > size(plannedWaypoints, 1)
            result.reachedAll = true;
            stopReason = "completed";
            stopDetails = sprintf('Completed all %d final-competition targets.', size(plannedWaypoints, 1));
            result.remainingGoals = zeros(0, 2);
            result.globalVisitOrder = zeros(0, 2);
            break;
        end

        localSetPowerLED(Robot, 'green', opts);
        currentPath = zeros(0, 2);
        pathTargetIdx = 1;
        snakeCount = 0;
        snakeRecoveryCounter = 0;
        prevDistToGoal = inf;
        prevCmdWSign = 0;
        result.remainingGoals = plannedWaypoints(targetIdx:end, :);
        result.globalVisitOrder = result.remainingGoals;
        pause(opts.loopPause);
        continue;
    end

    dynamicLookahead = opts.lookaheadDistance;
    dynamicShortcutLength = opts.maxShortcutLength;
    if distToGoal <= opts.nearGoalSlowRadius
        dynamicLookahead = min(dynamicLookahead, opts.nearGoalLookaheadDistance);
        dynamicShortcutLength = min(dynamicShortcutLength, opts.nearGoalShortcutLength);
    end
    if snakeRecoveryCounter > 0
        dynamicLookahead = min(dynamicLookahead, opts.nearGoalLookaheadDistance);
        dynamicShortcutLength = min(dynamicShortcutLength, opts.nearGoalShortcutLength);
        snakeRecoveryCounter = snakeRecoveryCounter - 1;
    end

    plannerOpts = struct('resolution', opts.planResolution, ...
                         'inflationRadius', opts.robotInflation, ...
                         'stayAwayPoints', stayAwayPoints, ...
                         'stayAwayRadius', opts.stayAwayInflation, ...
                         'preferredClearance', opts.preferredClearance, ...
                         'clearanceWeight', opts.clearanceWeight, ...
                         'maxShortcutLength', dynamicShortcutLength);

    if isempty(currentPath) || mod(iter, opts.replanEvery) == 1
        previousPath = currentPath;
        [candidatePath, ~, ~, found] = planPathAStar(navMap, boundary, poseCtrl(1:2)', currentTarget, plannerOpts);
        if found && ~isempty(candidatePath)
            currentPath = candidatePath;
            dataStore.plannedPath = currentPath;
            pathTargetIdx = 1;
        elseif isempty(previousPath)
            stopReason = "astar_no_initial_path";
            stopDetails = sprintf('A* failed to find an initial path from PF estimate to target %d.', targetIdx);
            warning('%s Stopping.', stopDetails);
            break;
        else
            warning('A* replanning failed at target %d. Continuing on previous path.', targetIdx);
        end
    end

    if opts.activeSenseSafetyEnabled
        safetyOpts = struct( ...
            'stopRange', opts.activeSenseStopRange, ...
            'forwardFov', opts.activeSenseForwardFov, ...
            'pathCorridor', opts.activeSensePathCorridor, ...
            'pathLookahead', opts.activeSensePathLookahead, ...
            'maxReliableRange', min(opts.activeSenseReliableRange, 2.0), ...
            'unexpectedMargin', opts.activeSenseUnexpectedMargin, ...
            'expectedDepth', depthPredict(poseCtrl, navMap, opts.sensorOrigin, angles, 10.0));
        [unsafe, unsafeReason, unsafePoint] = activeSenseSafetyCheck( ...
            poseCtrl, latestDepth, currentPath, opts.sensorOrigin, angles, safetyOpts);
        if unsafe
            SetFwdVelAngVelCreate(Robot, 0, 0);
            fprintf('Active sensing safety stop: %s\n', unsafeReason);
            dataStore.activeSenseEvents = [dataStore.activeSenseEvents; ...
                toc, targetIdx, unsafePoint(1), unsafePoint(2)];
            currentPath = zeros(0, 2);
            pathTargetIdx = 1;
            result.replanCount = result.replanCount + 1;
            activeReplanCooldown = opts.activeSenseReplanCooldown;
            pause(opts.loopPause);
            continue;
        end
    end

    while pathTargetIdx < size(currentPath, 1) && ...
            norm(currentPath(pathTargetIdx, :)' - poseCtrl(1:2)) < dynamicLookahead
        pathTargetIdx = pathTargetIdx + 1;
    end

    localTarget = currentPath(pathTargetIdx, :);
    delta = localTarget(:) - poseCtrl(1:2);
    distToLocal = norm(delta);

    if distToLocal < 1e-6
        desiredVel = [0; 0];
    else
        speed = opts.desiredSpeed;
        if distToGoal <= opts.nearGoalSlowRadius
            slowScale = max(distToGoal / opts.nearGoalSlowRadius, 0);
            speed = opts.nearGoalMinSpeed + ...
                (opts.desiredSpeed - opts.nearGoalMinSpeed) * slowScale;
        end
        if distToLocal < 0.4
            speed = speed * max(distToLocal / 0.4, 0.25);
        end
        desiredVel = speed * delta / distToLocal;
    end

    [cmdV, cmdW] = feedbackLin(desiredVel(1), desiredVel(2), poseCtrl(3), opts.epsilon);
    cmdV = max(min(cmdV, opts.maxForwardSpeed), -opts.maxReverseSpeed);
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, opts.maxWheelVelocity, opts.wheel2Center);
    cmdW = max(min(cmdW, opts.maxAngularSpeed), -opts.maxAngularSpeed);
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, opts.maxWheelVelocity, opts.wheel2Center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

    progressToGoal = prevDistToGoal - distToGoal;
    cmdWSign = sign(cmdW);
    isOscillating = abs(cmdW) >= opts.snakeAngularThreshold && ...
        cmdWSign ~= 0 && prevCmdWSign ~= 0 && cmdWSign ~= prevCmdWSign;
    isNotProgressing = isfinite(progressToGoal) && ...
        progressToGoal < opts.snakeProgressThreshold;
    if isNotProgressing && (isOscillating || abs(cmdW) >= opts.snakeAngularThreshold)
        snakeCount = snakeCount + 1;
    else
        snakeCount = max(0, snakeCount - 1);
    end
    if snakeCount >= opts.snakeTriggerCount
        fprintf(['Snake recovery: low progress with high angular command. ', ...
                 'target=%d dist=%.3f pathIdx=%d cmd=[%.3f %.3f]\n'], ...
            targetIdx, distToGoal, pathTargetIdx, cmdV, cmdW);
        currentPath = zeros(0, 2);
        pathTargetIdx = 1;
        snakeCount = 0;
        snakeRecoveryCounter = opts.snakeRecoverySteps;
    end
    prevDistToGoal = distToGoal;
    if cmdWSign ~= 0
        prevCmdWSign = cmdWSign;
    end

    if mod(iter, opts.plotEvery) == 0 && ~isempty(fig) && isvalid(fig)
        localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, opts.waypoints, targetIdx, ...
            currentPath, pathTargetIdx, particlesPre, dataStore);
    end

    if mod(iter, 10) == 0
        fprintf(['PF estimate: [x=%.3f, y=%.3f, th=%.3f], target=%d, ', ...
                 'goalDist=%.3f, pathIdx=%d, tags=%d, spread=%.3f, ', ...
                 'Neff=%.1f, odom=[%.3f %.3f], cmd=[%.3f %.3f], ', ...
                 'lookahead=%.2f, shortcut=%.2f, snake=%d\n'], ...
            poseCtrl(1), poseCtrl(2), poseCtrl(3), targetIdx, distToGoal, ...
            pathTargetIdx, size(tags, 1), pfSpread, neff, latestOdom(1), ...
            latestOdom(2), cmdV, cmdW, dynamicLookahead, dynamicShortcutLength, snakeCount);
    end

    pause(opts.loopPause);
end

SetFwdVelAngVelCreate(Robot, 0, 0);

if ~isempty(fig) && isvalid(fig)
    localUpdatePlot(fig, map, beaconLoc, stayAwayPoints, plannedWaypoints, ...
        min(targetIdx, size(plannedWaypoints, 1)), currentPath, pathTargetIdx, particlesPre, dataStore);
end

result.finalPoseEstimate = state.poseEstimate;
result.stopReason = stopReason;
result.stopDetails = stopDetails;
result.wallBeliefs = wallBeliefs;
result.activePlotData = activePlotData;
dataStore.visitedWaypoints = result.visitedWaypoints;
dataStore.wallBeliefs = wallBeliefs;
dataStore.activeNavMap = navMap;
[result.outputDataPath, result.outputPlotPath, result.outputFigurePath] = ...
    localSaveCompetitionOutput(baseDir, result, dataStore, mapStruct, plannedWaypoints);

if result.reachedAll
    fprintf('%s\n', result.stopDetails);
else
    fprintf('Stopped before completing all final-competition targets: %s\n', result.stopDetails);
end
fprintf('Final PF estimate: [x=%.3f, y=%.3f, th=%.3f]\n', ...
    result.finalPoseEstimate(1), result.finalPoseEstimate(2), result.finalPoseEstimate(3));
end

function plannedWaypoints = localBuildPlannedOrder(mapStruct, candidateStarts, goalPoints, initResult, opts)
plannedWaypoints = goalPoints;

if isempty(goalPoints)
    return;
end

stayAwayPoints = localOptionalField(mapStruct, 'StayAwayPoints', ...
    localOptionalField(mapStruct, 'stayAwayPoints', zeros(0, 2)));

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
    'preferredClearance', opts.preferredClearance, ...
    'clearanceWeight', opts.clearanceWeight, ...
    'maxShortcutLength', opts.maxShortcutLength, ...
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
    if isfield(mapStruct, 'knownMap')
        fprintf('Conservative global planning failed; retrying waypoint order on known map.\n');
        [~, costMatrixKnown, ~, nodeMetaKnown] = precomputePairwisePathCosts( ...
            mapStruct.knownMap, candidateStarts, goalPoints, plannerOpts);
        startNodeIdxKnown = nodeMetaKnown.candidateStartNodeIdx(initResult.bestWaypointIdx);
        remainingGoalNodeIdxKnown = nodeMetaKnown.goalNodeIdx(remainingGoalIdx);
        [~, visitGoalOrder, totalCost] = solveGlobalVisitOrder( ...
            costMatrixKnown, startNodeIdxKnown, remainingGoalNodeIdxKnown);
        if isempty(visitGoalOrder)
            visitGoalOrder = localGreedyVisitGoalOrder(costMatrixKnown, startNodeIdxKnown, remainingGoalNodeIdxKnown);
            totalCost = nan;
        end
    else
        visitGoalOrder = localGreedyVisitGoalOrder(costMatrix, startNodeIdx, remainingGoalNodeIdx);
        totalCost = nan;
    end
end
if isempty(visitGoalOrder)
    warning('finalCompetition:GlobalPlanningFailed', ...
        'Global planning failed; falling back to stored target order.');
    return;
end

plannedWaypoints = goalPoints(remainingGoalIdx(visitGoalOrder), :);
fprintf('Global planning cost: %.3f over %d remaining goals.\n', totalCost, size(plannedWaypoints, 1));
end

function plannedWaypoints = localReorderRemainingWaypoints(navMap, boundary, pose, visitedWaypoints, remainingGoals, stayAwayPoints, opts)
plannedWaypoints = [visitedWaypoints; remainingGoals];

if isempty(remainingGoals) || ~opts.useGlobalPlanning
    return;
end

plannerOpts = struct( ...
    'plannerType', opts.globalPlannerType, ...
    'boundary', boundary, ...
    'stayAwayPoints', stayAwayPoints, ...
    'planResolution', opts.planResolution, ...
    'robotInflation', opts.robotInflation, ...
    'stayAwayInflation', opts.stayAwayInflation, ...
    'preferredClearance', opts.preferredClearance, ...
    'clearanceWeight', opts.clearanceWeight, ...
    'maxShortcutLength', opts.maxShortcutLength, ...
    'nPRM', 150);

[~, costMatrix, ~, nodeMeta] = precomputePairwisePathCosts( ...
    navMap, pose(1:2)', remainingGoals, plannerOpts);

startNodeIdx = nodeMeta.candidateStartNodeIdx(1);
remainingGoalNodeIdx = nodeMeta.goalNodeIdx(1:size(remainingGoals, 1));
[~, visitGoalOrder, totalCost] = solveGlobalVisitOrder(costMatrix, startNodeIdx, remainingGoalNodeIdx);

if isempty(visitGoalOrder)
    visitGoalOrder = localGreedyVisitGoalOrder(costMatrix, startNodeIdx, remainingGoalNodeIdx);
    totalCost = nan;
    if isempty(visitGoalOrder)
        warning('finalCompetition:ActiveReplanFailed', ...
            'Active sensing global reorder failed; keeping previous remaining order.');
        return;
    end
end

plannedWaypoints = [visitedWaypoints; remainingGoals(visitGoalOrder, :)];
fprintf('Active sensing replanned remaining goals, cost %.3f over %d goals.\n', ...
    totalCost, size(remainingGoals, 1));
end

function visitGoalOrder = localGreedyVisitGoalOrder(costMatrix, startNodeIdx, remainingGoalNodeIdx)
remainingGoalNodeIdx = remainingGoalNodeIdx(:)';
unvisited = 1:numel(remainingGoalNodeIdx);
visitGoalOrder = zeros(1, 0);
currentNode = startNodeIdx;

while ~isempty(unvisited)
    costs = costMatrix(currentNode, remainingGoalNodeIdx(unvisited));
    [bestCost, bestLocalIdx] = min(costs);
    if ~isfinite(bestCost)
        visitGoalOrder = [visitGoalOrder, unvisited]; %#ok<AGROW>
        return;
    end

    nextGoalOrderIdx = unvisited(bestLocalIdx);
    visitGoalOrder(end + 1) = nextGoalOrderIdx; %#ok<AGROW>
    currentNode = remainingGoalNodeIdx(nextGoalOrderIdx);
    unvisited(bestLocalIdx) = [];
end
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

function controlPose = localUpdateControlPose(controlPose, pfPose, numVisibleTags, opts)
deltaXY = pfPose(1:2) - controlPose(1:2);
deltaTheta = localWrapToPi(pfPose(3) - controlPose(3));

if numVisibleTags > 0
    maxPoseStep = max(opts.maxPoseStepForControl, opts.tagSnapAlpha);
    maxThetaStep = max(opts.maxThetaStepForControl, opts.tagSnapAlpha);
else
    maxPoseStep = opts.maxPoseStepForControl;
    maxThetaStep = opts.maxThetaStepForControl;
end

if norm(deltaXY) > maxPoseStep
    deltaXY = deltaXY * (maxPoseStep / norm(deltaXY));
end
if abs(deltaTheta) > maxThetaStep
    deltaTheta = sign(deltaTheta) * maxThetaStep;
end

if numVisibleTags > 0
    alpha = max(opts.controlPoseAlphaWithTags, opts.tagSnapAlpha);
else
    alpha = opts.controlPoseAlpha;
end

controlPose(1:2) = controlPose(1:2) + alpha * deltaXY;
controlPose(3) = localWrapToPi(controlPose(3) + alpha * deltaTheta);
end

function [pfSpread, neff] = localPFHealth(particlesPre)
if isempty(particlesPre)
    pfSpread = 0;
    neff = inf;
    return;
end

w = particlesPre.weights;
p = particlesPre.poses;
pfSpread = sqrt(var(p(1, :)) + var(p(2, :)));
neff = 1 / sum(w .^ 2);
end

function processNoise = localScaledProcessNoise(odom, opts)
distanceStep = abs(odom(1));
headingStep = abs(odom(2));

positionSigma = opts.processNoisePositionBase + ...
    opts.processNoisePositionPerMeter * distanceStep;
thetaSigma = opts.processNoiseThetaBase + ...
    opts.processNoiseThetaPerRad * headingStep;

processNoise = diag([positionSigma ^ 2, positionSigma ^ 2, thetaSigma ^ 2]);
end

function reseed = localShouldReseedPF(pfSpread, neff, numVisibleTags, opts)
reseed = numVisibleTags > 0 && ...
    (pfSpread > opts.pfMaxSpreadBeforeReseed || neff < opts.pfMinNeffBeforeReseed);
end

function angleWrapped = localWrapToPi(angle)
angleWrapped = mod(angle + pi, 2 * pi) - pi;
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

function localSetPowerLED(Robot, colorName, opts)
switch lower(colorName)
    case 'green'
        powerColor = 0;
    case 'red'
        powerColor = 100;
    otherwise
        error('finalCompetition:InvalidLEDColor', ...
            'Unsupported power LED color ''%s''.', colorName);
end

SetLEDsRoomba(Robot, 3, powerColor, 100);
pause(opts.ledPause);
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

function [dataPath, plotPath, figPath] = localSaveCompetitionOutput(baseDir, result, dataStore, mapStruct, plannedWaypoints)
outputDir = fullfile(baseDir, 'src', 'output');
if ~isfolder(outputDir)
    mkdir(outputDir);
end

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
dataPath = string(fullfile(outputDir, "finalCompetitionOutput_" + timestamp + ".mat"));
plotPath = string(fullfile(outputDir, "finalCompetitionPlot_" + timestamp + ".png"));
figPath = string(fullfile(outputDir, "finalCompetitionPlot_" + timestamp + ".fig"));

result.outputDataPath = dataPath;
result.outputPlotPath = plotPath;
result.outputFigurePath = figPath;

outputData = struct();
outputData.robotPose = dataStore.robotPose;
outputData.odometry = dataStore.odometry;
outputData.depthData = dataStore.rsdepth;
outputData.bumpData = dataStore.bump;
outputData.beaconData = dataStore.beacon;
outputData.visitedWaypoints = result.visitedWaypoints;
outputData.wallBeliefs = result.wallBeliefs;
outputData.activeNavMap = dataStore.activeNavMap;
outputData.stopReason = result.stopReason;
outputData.stopDetails = result.stopDetails;
outputData.dataStore = dataStore;
outputData.result = result;
outputData.mapStruct = mapStruct;
outputData.plannedWaypoints = plannedWaypoints;
outputData.savedAt = timestamp;

save(char(dataPath), 'outputData', 'dataStore', 'result', 'mapStruct', 'plannedWaypoints');

try
    fig = localCreateFinalOutputPlot(mapStruct, plannedWaypoints, result, dataStore);
    savefig(fig, char(figPath));
    saveas(fig, char(plotPath));
    close(fig);
catch me
    warning('finalCompetition:OutputPlotFailed', ...
        'Saved output data but failed to save output plot: %s', me.message);
    plotPath = "";
    figPath = "";
end

fprintf('Saved competition output data: %s\n', char(dataPath));
if strlength(plotPath) > 0
    fprintf('Saved competition output plot: %s\n', char(plotPath));
end
end

function fig = localCreateFinalOutputPlot(mapStruct, plannedWaypoints, result, dataStore)
fig = figure('Name', 'Final Competition Output', 'Color', 'w', 'Visible', 'off');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');

map = mapStruct.map;
for i = 1:size(map, 1)
    plot(ax, [map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'k-', 'LineWidth', 2);
end

optWalls = localOptionalField(mapStruct, 'optWalls', zeros(0, 4));
localPlotOptionalWalls(ax, optWalls, result.wallBeliefs);

stayAwayPoints = localOptionalField(mapStruct, 'StayAwayPoints', ...
    localOptionalField(mapStruct, 'stayAwayPoints', zeros(0, 2)));
if ~isempty(stayAwayPoints)
    plot(ax, stayAwayPoints(:, 1), stayAwayPoints(:, 2), 'x', ...
        'Color', [1, 0.5, 0], 'MarkerSize', 8, 'LineWidth', 1.5);
end

if ~isempty(plannedWaypoints)
    plot(ax, plannedWaypoints(:, 1), plannedWaypoints(:, 2), 'bo', ...
        'MarkerFaceColor', 'c', 'MarkerSize', 7);
end

if ~isempty(result.visitedWaypoints)
    plot(ax, result.visitedWaypoints(:, 1), result.visitedWaypoints(:, 2), 'go', ...
        'MarkerFaceColor', 'g', 'MarkerSize', 9, 'LineWidth', 1.5);
end

if ~isempty(dataStore.robotPose)
    plot(ax, dataStore.robotPose(:, 2), dataStore.robotPose(:, 3), 'r-', 'LineWidth', 1.5);
    plot(ax, dataStore.robotPose(end, 2), dataStore.robotPose(end, 3), 'ro', ...
        'MarkerFaceColor', 'r');
end

xlim(ax, [min(map(:, [1, 3]), [], 'all') - 0.5, max(map(:, [1, 3]), [], 'all') + 0.5]);
ylim(ax, [min(map(:, [2, 4]), [], 'all') - 0.5, max(map(:, [2, 4]), [], 'all') + 0.5]);
title(ax, 'Final Competition Output');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
end

function localPlotOptionalWalls(ax, optWalls, wallBeliefs)
if isempty(optWalls)
    return;
end

for i = 1:size(optWalls, 1)
    color = 'r';
    lineStyle = '-';
    shouldPlot = true;
    if ~isempty(wallBeliefs) && numel(wallBeliefs) >= i
        if isstruct(wallBeliefs)
            status = wallBeliefs(i).status;
            probPresent = wallBeliefs(i).probPresent;
        else
            probPresent = wallBeliefs(i);
            if probPresent >= 0.7
                status = 'present';
            elseif probPresent <= 0.3
                status = 'absent';
            else
                status = 'unknown';
            end
        end

        if strcmp(status, 'present') || probPresent >= 0.7
            color = 'k';
        elseif strcmp(status, 'absent') || probPresent <= 0.3
            color = 'k';
            lineStyle = '--';
        end
    end

    if shouldPlot
        plot(ax, [optWalls(i, 1), optWalls(i, 3)], [optWalls(i, 2), optWalls(i, 4)], ...
            lineStyle, 'Color', color, 'LineWidth', 1.5);
    end
end
end

function value = localOptionalField(s, fieldName, defaultValue)
if isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
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

if isfile(mapMatPath) && endsWith(lower(mapMatPath), ".mat")
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
        if endsWith(lower(candidates(i)), ".mat")
            mapMatPath = candidates(i);
            return;
        end
    end
end

error('finalCompetition:MapNotFound', ...
    'Unable to find a .mat map for ''%s''.', mapMatPath);
end

function mapMatPath = localResolveMapDirectoryPath(baseDir)
mapDir = fullfile(baseDir, 'src', 'map');
if ~isfolder(mapDir)
    error('finalCompetition:MapDirectoryNotFound', ...
        'Expected map directory does not exist: %s', mapDir);
end

files = dir(mapDir);
files = files(~[files.isdir]);
files = files(~startsWith({files.name}, '.'));

if numel(files) ~= 1
    error('finalCompetition:InvalidMapDirectory', ...
        'Expected exactly one map file in %s, found %d.', mapDir, numel(files));
end

mapMatPath = localResolveMapPath(fullfile(files(1).folder, files(1).name));
end
