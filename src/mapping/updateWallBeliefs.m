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
if opts.usePFConfidenceGate && ...
        ((isfinite(opts.pfSpread) && opts.pfSpread > opts.pfMaxSpreadForUpdate) || ...
         (isfinite(opts.neff) && opts.neff < opts.pfMinNeffForUpdate))
    return;
end

[dKnown, ~] = depthPredictWithHits(robotPose, knownMap, sensorOrigin, angles, opts.maxRange);
[dOpt, hitOpt] = depthPredictWithHits(robotPose, optWalls, sensorOrigin, angles, opts.maxRange);
numWalls = numel(wallBeliefs);
residuals = cell(numWalls, 1);
wallPositions = cell(numWalls, 1);
knownAmbiguous = cell(numWalls, 1);
presentSupport = cell(numWalls, 1);
absentSupport = cell(numWalls, 1);

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

    wallPosition = localWallPosition(robotPose, sensorOrigin, angles(k), dOpt(k), optWalls(wallIdx, :));
    wallLength = norm(optWalls(wallIdx, 3:4) - optWalls(wallIdx, 1:2));
    if ~isfinite(wallPosition) || wallLength <= 0 || ...
            wallPosition < opts.endpointFilterDistance || ...
            wallPosition > wallLength - opts.endpointFilterDistance
        continue;
    end

    residual = z - dOpt(k);
    knownAlsoExplainsMeasurement = isfinite(dKnown(k)) && ...
        abs(z - dKnown(k)) <= opts.knownAmbiguityTolerance && ...
        abs(dKnown(k) - dOpt(k)) <= opts.knownAmbiguityRange;
    knownBackgroundVisible = isfinite(dKnown(k)) && dKnown(k) < opts.maxRange && ...
        dKnown(k) > dOpt(k) + opts.absentTolerance && abs(z - dKnown(k)) <= opts.backgroundTolerance;
    maxRangeBackgroundVisible = z >= min(opts.maxReliableRange, opts.maxRange) - opts.maxRangeTolerance;

    residuals{wallIdx}(end + 1, 1) = residual;
    wallPositions{wallIdx}(end + 1, 1) = wallPosition;
    knownAmbiguous{wallIdx}(end + 1, 1) = knownAlsoExplainsMeasurement;
    presentSupport{wallIdx}(end + 1, 1) = ...
        abs(residual) <= opts.presentTolerance && ~knownAlsoExplainsMeasurement;
    absentSupport{wallIdx}(end + 1, 1) = ...
        residual >= opts.absentTolerance && (knownBackgroundVisible || maxRangeBackgroundVisible);
end

for wallIdx = 1:numel(wallBeliefs)
    if opts.lockConfirmedWalls && ~strcmp(wallBeliefs(wallIdx).status, 'unknown')
        continue;
    end
    numFrameBeams = numel(residuals{wallIdx});
    if numFrameBeams < min(opts.minPresentFrameBeams, opts.minAbsentFrameBeams)
        continue;
    end
    wallCoverage = max(wallPositions{wallIdx}) - min(wallPositions{wallIdx});
    if mean(knownAmbiguous{wallIdx}) >= opts.knownAmbiguityMajorityRatio
        continue;
    end

    oldStatus = wallBeliefs(wallIdx).status;
    medianResidual = median(residuals{wallIdx});
    presentFrameEligible = numFrameBeams >= opts.minPresentFrameBeams && ...
        wallCoverage >= opts.minPresentWallCoverage;
    absentFrameEligible = numFrameBeams >= opts.minAbsentFrameBeams && ...
        wallCoverage >= opts.minAbsentWallCoverage;
    presentFrame = presentFrameEligible && ...
        mean(presentSupport{wallIdx}) >= opts.presentSupportRatio && ...
        abs(medianResidual) <= opts.presentTolerance;
    absentFrame = absentFrameEligible && ...
        mean(absentSupport{wallIdx}) >= opts.absentSupportRatio && ...
        medianResidual >= opts.absentTolerance;

    if presentFrame && ~absentFrame
        wallBeliefs(wallIdx).presentEvidenceCount = wallBeliefs(wallIdx).presentEvidenceCount + 1;
        wallBeliefs(wallIdx).absentEvidenceCount = max(0, wallBeliefs(wallIdx).absentEvidenceCount - 1);
        delta = opts.presentLogOdds;
    elseif absentFrame && ~presentFrame
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
    wallBeliefs(wallIdx).status = localStatus(oldStatus, wallBeliefs(wallIdx), opts);

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
if ~isfield(opts, 'knownAmbiguityMajorityRatio')
    opts.knownAmbiguityMajorityRatio = 0.50;
end
if ~isfield(opts, 'maxRangeTolerance')
    opts.maxRangeTolerance = 0.10;
end
if ~isfield(opts, 'presentLogOdds')
    opts.presentLogOdds = 1.2;
end
if ~isfield(opts, 'absentLogOdds')
    opts.absentLogOdds = 0.55;
end
if ~isfield(opts, 'presentThreshold')
    opts.presentThreshold = 0.75;
end
if ~isfield(opts, 'absentThreshold')
    opts.absentThreshold = 0.15;
end
if ~isfield(opts, 'maxLogOdds')
    opts.maxLogOdds = 6.0;
end
if ~isfield(opts, 'lockConfirmedWalls')
    opts.lockConfirmedWalls = false;
end
if ~isfield(opts, 'minPresentEvidence')
    opts.minPresentEvidence = 2;
end
if ~isfield(opts, 'minAbsentEvidence')
    opts.minAbsentEvidence = 5;
end
if ~isfield(opts, 'endpointFilterDistance')
    opts.endpointFilterDistance = 0.15;
end
if ~isfield(opts, 'minFrameBeams')
    opts.minFrameBeams = 3;
end
if ~isfield(opts, 'minPresentFrameBeams')
    opts.minPresentFrameBeams = min(2, opts.minFrameBeams);
end
if ~isfield(opts, 'minAbsentFrameBeams')
    opts.minAbsentFrameBeams = opts.minFrameBeams;
end
if ~isfield(opts, 'minWallCoverage')
    opts.minWallCoverage = 0.20;
end
if ~isfield(opts, 'minPresentWallCoverage')
    opts.minPresentWallCoverage = min(0.10, opts.minWallCoverage);
end
if ~isfield(opts, 'minAbsentWallCoverage')
    opts.minAbsentWallCoverage = opts.minWallCoverage;
end
if ~isfield(opts, 'presentSupportRatio')
    opts.presentSupportRatio = 0.50;
end
if ~isfield(opts, 'absentSupportRatio')
    opts.absentSupportRatio = 0.70;
end
if ~isfield(opts, 'usePFConfidenceGate')
    opts.usePFConfidenceGate = false;
end
if ~isfield(opts, 'pfSpread')
    opts.pfSpread = 0;
end
if ~isfield(opts, 'neff')
    opts.neff = inf;
end
if ~isfield(opts, 'pfMaxSpreadForUpdate')
    opts.pfMaxSpreadForUpdate = 0.12;
end
if ~isfield(opts, 'pfMinNeffForUpdate')
    opts.pfMinNeffForUpdate = 60;
end
if ~isfield(opts, 'presentToUnknownThreshold')
    opts.presentToUnknownThreshold = 0.45;
end
if ~isfield(opts, 'absentToUnknownThreshold')
    opts.absentToUnknownThreshold = 0.55;
end
end

function status = localStatus(oldStatus, wallBelief, opts)
status = oldStatus;

switch oldStatus
    case 'present'
        if wallBelief.probPresent <= opts.presentToUnknownThreshold && ...
                wallBelief.absentEvidenceCount >= opts.minAbsentEvidence
            status = 'unknown';
        end
    case 'absent'
        if wallBelief.probPresent >= opts.absentToUnknownThreshold && ...
                wallBelief.presentEvidenceCount >= opts.minPresentEvidence
            status = 'unknown';
        end
    otherwise
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
end

function wallPosition = localWallPosition(robotPose, sensorOrigin, angle, rangeToWall, wall)
wallPosition = nan;
if ~isfinite(rangeToWall)
    return;
end

theta = robotPose(3);
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
sensorGlobal = robotPose(1:2) + R * sensorOrigin(:);
hitPoint = sensorGlobal + rangeToWall * [cos(theta + angle); sin(theta + angle)];

p1 = wall(1:2)';
p2 = wall(3:4)';
wallVec = p2 - p1;
wallLength = norm(wallVec);
if wallLength <= 0
    return;
end

wallPosition = dot(hitPoint - p1, wallVec / wallLength);
end
