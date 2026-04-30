function testActiveSensingLogic()
% TESTACTIVESENSINGLOGIC Basic synthetic checks for optional-wall mapping.

knownMap = [
    0, -1, 4, -1
    4, -1, 4,  1
    4,  1, 0,  1
    0,  1, 0, -1
];
optWalls = [2, -0.5, 2, 0.5];
pose = [1; 0; 0];
sensorOrigin = [0; 0];
angles = [-0.2; 0; 0.2];

opts = struct('presentThreshold', 0.70, 'absentThreshold', 0.35, ...
    'presentTolerance', 0.15, 'absentTolerance', 0.25, ...
    'backgroundTolerance', 0.30, 'maxReliableRange', 1.2, ...
    'minPresentEvidence', 4, 'minAbsentEvidence', 5, ...
    'minFrameBeams', 3, 'minWallCoverage', 0.20, ...
    'endpointFilterDistance', 0.12);

presentDepth = depthPredict(pose, optWalls, sensorOrigin, angles, 10.0);
absentDepth = depthPredict(pose, knownMap, sensorOrigin, angles, 10.0);

fastPresentOpts = opts;
fastPresentOpts.minPresentEvidence = 2;
fastPresentOpts.minAbsentEvidence = 5;
fastPresentOpts.minPresentFrameBeams = 2;
fastPresentOpts.minAbsentFrameBeams = 3;
fastPresentOpts.minPresentWallCoverage = 0.10;
fastPresentOpts.minAbsentWallCoverage = 0.20;
fastPresentOpts.presentSupportRatio = 0.50;
fastPresentOpts.presentLogOdds = 1.2;
fastPresentOpts.presentThreshold = 0.75;
wallBeliefs = initWallBeliefs(optWalls);
[wallBeliefs, changed] = updateWallBeliefs( ...
    pose, presentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, fastPresentOpts);
assert(~changed);
[wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
    pose, presentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, fastPresentOpts);
assert(changed);
assert(presentChanged);
assert(~absentChanged);
assert(strcmp(wallBeliefs(1).status, 'present'));

wallBeliefs = initWallBeliefs(optWalls);
for i = 1:4
    [wallBeliefs, changed] = updateWallBeliefs( ...
        pose, absentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, fastPresentOpts);
    assert(~changed);
end
assert(strcmp(wallBeliefs(1).status, 'unknown'));

wallBeliefs = initWallBeliefs(optWalls);
for i = 1:3
    [wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, presentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
    assert(~changed);
    assert(~presentChanged);
    assert(~absentChanged);
    assert(strcmp(wallBeliefs(1).status, 'unknown'));
end
[wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, presentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
assert(changed);
assert(presentChanged);
assert(~absentChanged);
assert(strcmp(wallBeliefs(1).status, 'present'));

wallBeliefs = initWallBeliefs(optWalls);
for i = 1:4
    [wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, absentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
    assert(~changed);
    assert(~presentChanged);
    assert(~absentChanged);
    assert(strcmp(wallBeliefs(1).status, 'unknown'));
end
[wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, absentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
assert(changed);
assert(~presentChanged);
assert(absentChanged);
assert(strcmp(wallBeliefs(1).status, 'absent'));

singleBeamOpts = opts;
singleBeamOpts.minFrameBeams = 3;
wallBeliefs = initWallBeliefs(optWalls);
[wallBeliefs, changed] = updateWallBeliefs( ...
    pose, presentDepth(2), knownMap, optWalls, wallBeliefs, sensorOrigin, angles(2), singleBeamOpts);
assert(~changed);
assert(strcmp(wallBeliefs(1).status, 'unknown'));

occludingKnown = [1.5, -0.5, 1.5, 0.5];
wallBeliefs = initWallBeliefs(optWalls);
[wallBeliefs, changed] = updateWallBeliefs( ...
    pose, depthPredict(pose, occludingKnown, sensorOrigin, angles, 10.0), ...
    occludingKnown, optWalls, wallBeliefs, sensorOrigin, angles, opts);
assert(~changed);
assert(strcmp(wallBeliefs(1).status, 'unknown'));

nearBehindKnown = [2.08, -1, 2.08, 1];
wallBeliefs = initWallBeliefs(optWalls);
for i = 1:4
    [wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, depthPredict(pose, nearBehindKnown, sensorOrigin, angles, 10.0), ...
        nearBehindKnown, optWalls, wallBeliefs, sensorOrigin, angles, opts);
    assert(~changed);
    assert(~presentChanged);
    assert(~absentChanged);
    assert(strcmp(wallBeliefs(1).status, 'unknown'));
end

endpointOptWall = [2, -0.08, 2, 0.92];
cornerKnown = [2.04, -1, 2.04, 1];
endpointPose = [1; -0.08; 0];
endpointAngles = [-0.04; 0; 0.04];
wallBeliefs = initWallBeliefs(endpointOptWall);
for i = 1:4
    [wallBeliefs, changed] = updateWallBeliefs( ...
        endpointPose, depthPredict(endpointPose, cornerKnown, sensorOrigin, endpointAngles, 10.0), ...
        cornerKnown, endpointOptWall, wallBeliefs, sensorOrigin, endpointAngles, opts);
    assert(~changed);
    assert(strcmp(wallBeliefs(1).status, 'unknown'));
end

pfOpts = opts;
pfOpts.usePFConfidenceGate = true;
pfOpts.pfSpread = 0.30;
pfOpts.neff = 20;
wallBeliefs = initWallBeliefs(optWalls);
for i = 1:5
    [wallBeliefs, changed] = updateWallBeliefs( ...
        pose, presentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, pfOpts);
    assert(~changed);
    assert(strcmp(wallBeliefs(1).status, 'unknown'));
end

wallBeliefs = initWallBeliefs(optWalls);
for i = 1:4
    [wallBeliefs, ~] = updateWallBeliefs( ...
        pose, presentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
end
assert(strcmp(wallBeliefs(1).status, 'present'));
for i = 1:5
    [wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, absentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
end
assert(~strcmp(wallBeliefs(1).status, 'absent'));
assert(~presentChanged);
assert(~absentChanged);
assert(~changed || strcmp(wallBeliefs(1).status, 'unknown'));

wallBeliefs = initWallBeliefs(optWalls);
[navMap, plotData] = buildActiveNavMap(knownMap, optWalls, wallBeliefs, 'conservative');
assert(size(navMap, 1) == size(knownMap, 1) + 1);
assert(size(plotData.unknown, 1) == 1);

[navMap, plotData] = buildActiveNavMap(knownMap, optWalls, wallBeliefs);
assert(size(navMap, 1) == size(knownMap, 1));
assert(size(plotData.unknown, 1) == 1);

wallBeliefs = initWallBeliefs(optWalls);
for i = 1:4
    [wallBeliefs, ~] = updateWallBeliefs( ...
        pose, presentDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
end
[navMap, plotData] = buildActiveNavMap(knownMap, optWalls, wallBeliefs);
assert(size(navMap, 1) == size(knownMap, 1) + 1);
assert(size(plotData.present, 1) == 1);

safetyOpts = struct('stopRange', 0.4, 'forwardFov', 12 * pi / 180, ...
    'pathCorridor', 0.2, 'pathLookahead', 1.0, 'maxReliableRange', 2.0);
[unsafe, ~, ~] = activeSenseSafetyCheck( ...
    pose, 0.3, [1, 0; 3, 0], sensorOrigin, 0, safetyOpts);
assert(unsafe);

[unsafe, ~, hitPoint] = activeSenseSafetyCheck( ...
    pose, 0.8, [1, 0; 3, 0], sensorOrigin, 0, safetyOpts);
assert(unsafe);
assert(all(isfinite(hitPoint)));

safetyOpts.expectedDepth = 0.9;
[unsafe, ~, ~] = activeSenseSafetyCheck( ...
    pose, 0.8, [1, 0; 3, 0], sensorOrigin, 0, safetyOpts);
assert(~unsafe);

fprintf('testActiveSensingLogic passed.\n');
end
