function [unsafe, reason, hitPoint] = activeSenseSafetyCheck(pose, zDepth, currentPath, sensorOrigin, angles, opts)
% ACTIVESENSESAFETYCHECK Stop if RealSense sees an unexpected obstacle.

if nargin < 6
    opts = struct();
end
opts = localDefaults(opts);

unsafe = false;
reason = '';
hitPoint = [nan, nan];

if isempty(zDepth)
    return;
end

zDepth = zDepth(:);
angles = angles(:);
n = min(length(zDepth), length(angles));
zDepth = zDepth(1:n);
angles = angles(1:n);
expectedDepth = opts.expectedDepth(:);
expectedDepth = expectedDepth(1:min(length(expectedDepth), n));
if length(expectedDepth) < n
    expectedDepth(end + 1:n, 1) = opts.maxReliableRange;
end

valid = isfinite(zDepth) & zDepth > 0 & isfinite(expectedDepth);
unexpected = valid & (zDepth < expectedDepth - opts.unexpectedMargin);

forwardMask = abs(angles) <= opts.forwardFov;
forwardDepth = zDepth(forwardMask & unexpected);
forwardDepth = forwardDepth(isfinite(forwardDepth) & forwardDepth > 0);
if ~isempty(forwardDepth) && min(forwardDepth) <= opts.stopRange
    unsafe = true;
    reason = 'front depth below stop range';
    return;
end

if isempty(currentPath) || size(currentPath, 1) < 2
    return;
end

points = localDepthPointsGlobal(pose, zDepth(unexpected), sensorOrigin, angles(unexpected), opts.maxReliableRange);
if isempty(points)
    return;
end

for i = 1:size(points, 1)
    [nearPath, alongDist] = localPointNearPath(points(i, :), pose(1:2)', currentPath, opts);
    if nearPath && alongDist <= opts.pathLookahead
        unsafe = true;
        reason = 'depth point inside active path corridor';
        hitPoint = points(i, :);
        return;
    end
end
end

function opts = localDefaults(opts)
if ~isfield(opts, 'stopRange')
    opts.stopRange = 0.34;
end
if ~isfield(opts, 'forwardFov')
    opts.forwardFov = 12 * pi / 180;
end
if ~isfield(opts, 'pathCorridor')
    opts.pathCorridor = 0.22;
end
if ~isfield(opts, 'pathLookahead')
    opts.pathLookahead = 0.55;
end
if ~isfield(opts, 'maxReliableRange')
    opts.maxReliableRange = 2.0;
end
if ~isfield(opts, 'unexpectedMargin')
    opts.unexpectedMargin = 0.18;
end
if ~isfield(opts, 'expectedDepth')
    opts.expectedDepth = opts.maxReliableRange + opts.unexpectedMargin + 1.0;
end
end

function points = localDepthPointsGlobal(pose, zDepth, sensorOrigin, angles, maxRange)
valid = isfinite(zDepth) & zDepth > 0 & zDepth <= maxRange;
zDepth = zDepth(valid);
angles = angles(valid);

if isempty(zDepth)
    points = zeros(0, 2);
    return;
end

theta = pose(3);
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
sensorGlobal = pose(1:2) + R * sensorOrigin(:);
pointsSensor = [zDepth(:)' .* cos(angles(:)'); zDepth(:)' .* sin(angles(:)')];
pointsGlobal = sensorGlobal + R * pointsSensor;
points = pointsGlobal';
end

function [nearPath, alongDist] = localPointNearPath(point, robotXY, path, opts)
nearPath = false;
alongDist = inf;
accumulated = 0;

for i = 1:size(path, 1) - 1
    a = path(i, :);
    b = path(i + 1, :);
    ab = b - a;
    segLen = norm(ab);
    if segLen < 1e-9
        continue;
    end

    t = max(0, min(1, dot(point - a, ab) / dot(ab, ab)));
    projection = a + t * ab;
    dist = norm(point - projection);
    candidateAlong = accumulated + t * segLen;

    if dist <= opts.pathCorridor && norm(projection - robotXY) <= opts.pathLookahead
        nearPath = true;
        alongDist = candidateAlong;
        return;
    end

    accumulated = accumulated + segLen;
end
end
