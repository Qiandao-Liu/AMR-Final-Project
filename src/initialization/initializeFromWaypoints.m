function result = initializeFromWaypoints(mapStruct, zMeasured, headingHypotheses, sensorOrigin, angles, opts)
% INITIALIZEFROMWAYPOINTS Pick the best start pose from waypoint candidates.
%
%   result = initializeFromWaypoints(mapStruct, zMeasured, headingHypotheses, ...
%       sensorOrigin, angles, opts)
%
%   This function is tailored to the final competition data format where
%   the start position is guaranteed to be one of mapStruct.waypoints and
%   the initial heading is unknown.
%
%   INPUTS
%       mapStruct          struct with fields:
%                            .map        known walls (Nx4)
%                            .waypoints  candidate start positions (Mx2)
%       zMeasured          Kx1 measured depth vector
%       headingHypotheses  1xH or Hx1 candidate headings in radians
%       sensorOrigin       2x1 depth sensor origin in robot frame
%       angles             Kx1 beam angles in sensor frame
%       opts               optional scoring options for scorePoseHypotheses
%
%   OUTPUT
%       result             struct with fields:
%                            .bestPose
%                            .bestWaypointIdx
%                            .bestHeadingIdx
%                            .scores
%                            .candidatePoses

if nargin < 6
    opts = struct();
end

waypoints = mapStruct.waypoints;
headings = headingHypotheses(:)';

numWaypoints = size(waypoints, 1);
numHeadings = length(headings);

candidatePoses = zeros(3, numWaypoints * numHeadings);
candidateMeta = zeros(numWaypoints * numHeadings, 2);

idx = 1;
for i = 1:numWaypoints
    for j = 1:numHeadings
        candidatePoses(:, idx) = [waypoints(i, 1); waypoints(i, 2); headings(j)];
        candidateMeta(idx, :) = [i, j];
        idx = idx + 1;
    end
end

scores = scorePoseHypotheses(mapStruct.map, candidatePoses, zMeasured, sensorOrigin, angles, opts);
[~, bestIdx] = max(scores);

result.bestPose = candidatePoses(:, bestIdx);
result.bestWaypointIdx = candidateMeta(bestIdx, 1);
result.bestHeadingIdx = candidateMeta(bestIdx, 2);
result.scores = reshape(scores, numHeadings, numWaypoints)';
result.candidatePoses = candidatePoses;
end
