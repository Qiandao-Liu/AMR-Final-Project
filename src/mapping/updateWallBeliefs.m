function [wallBeliefs, changed] = updateWallBeliefs(robotPose, zDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts)
% UPDATEWALLBELIEFS Update optional-wall status from RealSense depth.

if nargin < 8
    opts = struct();
end
opts = localDefaults(opts);

changed = false;
if isempty(optWalls) || isempty(wallBeliefs)
    return;
end

zDepth = zDepth(:);
[dKnown, ~] = depthPredictWithHits(robotPose, knownMap, sensorOrigin, angles, opts.maxRange);
[dOpt, hitOpt] = depthPredictWithHits(robotPose, optWalls, sensorOrigin, angles, opts.maxRange);

for k = 1:min(length(zDepth), length(angles))
    wallIdx = hitOpt(k);
    if wallIdx == 0
        continue;
    end
    if opts.lockConfirmedWalls && ~strcmp(wallBeliefs(wallIdx).status, 'unknown')
        continue;
    end

    z = zDepth(k);
    if ~isfinite(z) || z <= 0 || dOpt(k) > opts.maxReliableRange
        continue;
    end

    % A known wall in front of the optional wall occludes this observation.
    if dKnown(k) < dOpt(k) - opts.occlusionMargin
        continue;
    end

    residual = z - dOpt(k);
    if abs(residual) <= opts.presentTolerance
        delta = opts.presentLogOdds;
    elseif residual >= opts.absentTolerance
        delta = -opts.absentLogOdds;
    else
        continue;
    end

    oldStatus = wallBeliefs(wallIdx).status;
    wallBeliefs(wallIdx).logOdds = max(min( ...
        wallBeliefs(wallIdx).logOdds + delta, opts.maxLogOdds), -opts.maxLogOdds);
    wallBeliefs(wallIdx).probPresent = 1 / (1 + exp(-wallBeliefs(wallIdx).logOdds));
    wallBeliefs(wallIdx).observationCount = wallBeliefs(wallIdx).observationCount + 1;
    wallBeliefs(wallIdx).status = localStatus(wallBeliefs(wallIdx).probPresent, opts);

    if ~strcmp(oldStatus, wallBeliefs(wallIdx).status)
        changed = true;
    end
end
end

function opts = localDefaults(opts)
if ~isfield(opts, 'maxRange')
    opts.maxRange = 10.0;
end
if ~isfield(opts, 'maxReliableRange')
    opts.maxReliableRange = 4.0;
end
if ~isfield(opts, 'occlusionMargin')
    opts.occlusionMargin = 0.05;
end
if ~isfield(opts, 'presentTolerance')
    opts.presentTolerance = 0.18;
end
if ~isfield(opts, 'absentTolerance')
    opts.absentTolerance = 0.28;
end
if ~isfield(opts, 'presentLogOdds')
    opts.presentLogOdds = 1.1;
end
if ~isfield(opts, 'absentLogOdds')
    opts.absentLogOdds = 0.8;
end
if ~isfield(opts, 'presentThreshold')
    opts.presentThreshold = 0.85;
end
if ~isfield(opts, 'absentThreshold')
    opts.absentThreshold = 0.15;
end
if ~isfield(opts, 'maxLogOdds')
    opts.maxLogOdds = 6.0;
end
if ~isfield(opts, 'lockConfirmedWalls')
    opts.lockConfirmedWalls = true;
end
end

function status = localStatus(probPresent, opts)
if probPresent >= opts.presentThreshold
    status = 'present';
elseif probPresent <= opts.absentThreshold
    status = 'absent';
else
    status = 'unknown';
end
end
