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
angles = 0;

opts = struct('presentThreshold', 0.70, 'absentThreshold', 0.35, ...
    'presentTolerance', 0.15, 'absentTolerance', 0.25, ...
    'backgroundTolerance', 0.30, 'maxReliableRange', 1.0, ...
    'minPresentEvidence', 3, 'minAbsentEvidence', 3);

wallBeliefs = initWallBeliefs(optWalls);
for i = 1:2
    [wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, 1.0, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
    assert(~changed);
    assert(~presentChanged);
    assert(~absentChanged);
    assert(strcmp(wallBeliefs(1).status, 'unknown'));
end
[wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, 1.0, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
assert(changed);
assert(presentChanged);
assert(~absentChanged);
assert(strcmp(wallBeliefs(1).status, 'present'));

wallBeliefs = initWallBeliefs(optWalls);
for i = 1:2
    [wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, 3.0, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
    assert(~changed);
    assert(~presentChanged);
    assert(~absentChanged);
    assert(strcmp(wallBeliefs(1).status, 'unknown'));
end
[wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, 3.0, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
assert(changed);
assert(~presentChanged);
assert(absentChanged);
assert(strcmp(wallBeliefs(1).status, 'absent'));
[wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
    pose, 1.0, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
assert(~changed);
assert(~presentChanged);
assert(~absentChanged);
assert(strcmp(wallBeliefs(1).status, 'absent'));

occludingKnown = [1.5, -0.5, 1.5, 0.5];
wallBeliefs = initWallBeliefs(optWalls);
[wallBeliefs, changed] = updateWallBeliefs( ...
    pose, 1.5, occludingKnown, optWalls, wallBeliefs, sensorOrigin, angles, opts);
assert(~changed);
assert(strcmp(wallBeliefs(1).status, 'unknown'));

nearBehindKnown = [2.08, -1, 2.08, 1];
wallBeliefs = initWallBeliefs(optWalls);
for i = 1:3
    [wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
        pose, 1.08, nearBehindKnown, optWalls, wallBeliefs, sensorOrigin, angles, opts);
    assert(~changed);
    assert(~presentChanged);
    assert(~absentChanged);
    assert(strcmp(wallBeliefs(1).status, 'unknown'));
end

wallBeliefs = initWallBeliefs(optWalls);
[wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs( ...
    [0; 0; 0], 2.0, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
assert(~changed);
assert(~presentChanged);
assert(~absentChanged);
assert(strcmp(wallBeliefs(1).status, 'unknown'));

[navMap, plotData] = buildActiveNavMap(knownMap, optWalls, wallBeliefs, 'conservative');
assert(size(navMap, 1) == size(knownMap, 1) + 1);
assert(size(plotData.unknown, 1) == 1);

[navMap, plotData] = buildActiveNavMap(knownMap, optWalls, wallBeliefs);
assert(size(navMap, 1) == size(knownMap, 1));
assert(size(plotData.unknown, 1) == 1);

wallBeliefs = initWallBeliefs(optWalls);
for i = 1:3
    [wallBeliefs, ~] = updateWallBeliefs( ...
        pose, 1.0, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts);
end
[navMap, plotData] = buildActiveNavMap(knownMap, optWalls, wallBeliefs);
assert(size(navMap, 1) == size(knownMap, 1) + 1);
assert(size(plotData.present, 1) == 1);

safetyOpts = struct('stopRange', 0.4, 'forwardFov', 12 * pi / 180, ...
    'pathCorridor', 0.2, 'pathLookahead', 1.0, 'maxReliableRange', 2.0);
[unsafe, ~, ~] = activeSenseSafetyCheck( ...
    pose, 0.3, [1, 0; 3, 0], sensorOrigin, angles, safetyOpts);
assert(unsafe);

[unsafe, ~, hitPoint] = activeSenseSafetyCheck( ...
    pose, 0.8, [1, 0; 3, 0], sensorOrigin, angles, safetyOpts);
assert(unsafe);
assert(all(isfinite(hitPoint)));

safetyOpts.expectedDepth = 0.9;
[unsafe, ~, ~] = activeSenseSafetyCheck( ...
    pose, 0.8, [1, 0; 3, 0], sensorOrigin, angles, safetyOpts);
assert(~unsafe);

fprintf('testActiveSensingLogic passed.\n');
end
