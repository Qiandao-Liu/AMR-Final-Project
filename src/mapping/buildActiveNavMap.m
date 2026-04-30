function [navMap, plotData] = buildActiveNavMap(knownMap, optWalls, wallBeliefs, mode)
% BUILDACTIVENAVMAP Build planner map from known walls and wall beliefs.

if nargin < 4 || isempty(mode)
    mode = 'confirmed';
end

plotData = struct('known', knownMap, 'present', zeros(0, 4), ...
    'unknown', zeros(0, 4), 'absent', zeros(0, 4));

if isempty(optWalls) || isempty(wallBeliefs)
    navMap = knownMap;
    return;
end

statuses = {wallBeliefs.status};
presentMask = strcmp(statuses, 'present');
unknownMask = strcmp(statuses, 'unknown');
absentMask = strcmp(statuses, 'absent');

plotData.present = optWalls(presentMask, :);
plotData.unknown = optWalls(unknownMask, :);
plotData.absent = optWalls(absentMask, :);

switch lower(mode)
    case 'confirmed'
        navMap = [knownMap; plotData.present];
    otherwise
        navMap = [knownMap; plotData.present; plotData.unknown];
end
end
