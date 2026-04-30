function [wallBeliefs, changed, presentChanged, absentChanged] = updateWallBeliefs(robotPose, zDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles, opts)
% UPDATEWALLBELIEFS Update optional-wall status from RealSense depth.

if nargin < 8
    opts = struct();
end
opts = localDefaults(opts);

changed = false;
presentChanged = false;
absentChanged = false;
if isempty(optWalls) || isempty(wallBeliefs)
    return;
end

zDepth = zDepth(:);
[dKnown, ~] = depthPredictWithHits(robotPose, knownMap, sensorOrigin, angles, opts.maxRange);
[dOpt, hitOpt] = depthPredictWithHits(robotPose, optWalls, sensorOrigin, angles, opts.maxRange);
presentEvidence = false(size(wallBeliefs));
absentEvidence = false(size(wallBeliefs));
hasObservation = false(size(wallBeliefs));

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
    hasObservation(wallIdx) = true;

    residual = z - dOpt(k);
    if abs(residual) <= opts.presentTolerance
        knownAlsoExplainsMeasurement = isfinite(dKnown(k)) && ...
            abs(z - dKnown(k)) <= opts.knownAmbiguityTolerance && ...
            abs(dKnown(k) - dOpt(k)) <= opts.knownAmbiguityRange;
        if ~knownAlsoExplainsMeasurement
            presentEvidence(wallIdx) = true;
        end
    elseif residual >= opts.absentTolerance
        knownBackgroundVisible = isfinite(dKnown(k)) && dKnown(k) < opts.maxRange && ...
            dKnown(k) > dOpt(k) + opts.absentTolerance && abs(z - dKnown(k)) <= opts.backgroundTolerance;
        maxRangeBackgroundVisible = z >= min(opts.maxReliableRange, opts.maxRange) - opts.maxRangeTolerance;
        if knownBackgroundVisible || maxRangeBackgroundVisible
            absentEvidence(wallIdx) = true;
        end
    end
end

for wallIdx = 1:numel(wallBeliefs)
    if opts.lockConfirmedWalls && ~strcmp(wallBeliefs(wallIdx).status, 'unknown')
        continue;
    end
    if ~hasObservation(wallIdx)
        continue;
    end

    oldStatus = wallBeliefs(wallIdx).status;
    if presentEvidence(wallIdx) && ~absentEvidence(wallIdx)
        wallBeliefs(wallIdx).presentEvidenceCount = wallBeliefs(wallIdx).presentEvidenceCount + 1;
        wallBeliefs(wallIdx).absentEvidenceCount = max(0, wallBeliefs(wallIdx).absentEvidenceCount - 1);
        delta = opts.presentLogOdds;
    elseif absentEvidence(wallIdx) && ~presentEvidence(wallIdx)
        wallBeliefs(wallIdx).absentEvidenceCount = wallBeliefs(wallIdx).absentEvidenceCount + 1;
        wallBeliefs(wallIdx).presentEvidenceCount = max(0, wallBeliefs(wallIdx).presentEvidenceCount - 1);
        delta = -opts.absentLogOdds;
    else
        continue;
    end

    wallBeliefs(wallIdx).logOdds = max(min(wallBeliefs(wallIdx).logOdds + delta, ...
        opts.maxLogOdds), -opts.maxLogOdds);
    wallBeliefs(wallIdx).probPresent = 1 / (1 + exp(-wallBeliefs(wallIdx).logOdds));
    wallBeliefs(wallIdx).observationCount = wallBeliefs(wallIdx).observationCount + 1;
    wallBeliefs(wallIdx).status = localStatus(wallBeliefs(wallIdx), opts);

    if ~strcmp(oldStatus, wallBeliefs(wallIdx).status)
        changed = true;
        if strcmp(wallBeliefs(wallIdx).status, 'present')
            presentChanged = true;
        elseif strcmp(wallBeliefs(wallIdx).status, 'absent')
            absentChanged = true;
        end
    end
end
end

function opts = localDefaults(opts)
if ~isfield(opts, 'maxRange')
    opts.maxRange = 10.0;
end
if ~isfield(opts, 'maxReliableRange')
    opts.maxReliableRange = 1.0;
end
if ~isfield(opts, 'occlusionMargin')
    opts.occlusionMargin = 0.05;
end
if ~isfield(opts, 'presentTolerance')
    opts.presentTolerance = 0.12;
end
if ~isfield(opts, 'absentTolerance')
    opts.absentTolerance = 0.35;
end
if ~isfield(opts, 'backgroundTolerance')
    opts.backgroundTolerance = 0.25;
end
if ~isfield(opts, 'knownAmbiguityTolerance')
    opts.knownAmbiguityTolerance = 0.18;
end
if ~isfield(opts, 'knownAmbiguityRange')
    opts.knownAmbiguityRange = 0.30;
end
if ~isfield(opts, 'maxRangeTolerance')
    opts.maxRangeTolerance = 0.10;
end
if ~isfield(opts, 'presentLogOdds')
    opts.presentLogOdds = 0.7;
end
if ~isfield(opts, 'absentLogOdds')
    opts.absentLogOdds = 0.55;
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
if ~isfield(opts, 'minPresentEvidence')
    opts.minPresentEvidence = 3;
end
if ~isfield(opts, 'minAbsentEvidence')
    opts.minAbsentEvidence = 3;
end
end

function status = localStatus(wallBelief, opts)
if wallBelief.probPresent >= opts.presentThreshold && ...
        wallBelief.presentEvidenceCount >= opts.minPresentEvidence
    status = 'present';
elseif wallBelief.probPresent <= opts.absentThreshold && ...
        wallBelief.absentEvidenceCount >= opts.minAbsentEvidence
    status = 'absent';
else
    status = 'unknown';
end
end
