function [nodes, costMatrix, pathCache, nodeMeta] = precomputePairwisePathCosts(navMap, candidateStarts, goalPoints, plannerOpts)
% PRECOMPUTEPAIRWISEPATHCOSTS Compute pairwise feasible path costs on a
% navigation map for all candidate starts and goals.

if nargin < 4
    plannerOpts = struct();
end

allPoints = [candidateStarts; goalPoints];
[nodes, ~, allToNodeIdx] = unique(allPoints, 'rows', 'stable');

numStart = size(candidateStarts, 1);
numGoal = size(goalPoints, 1);
numNodes = size(nodes, 1);

candidateStartNodeIdx = allToNodeIdx(1:numStart);
goalNodeIdx = allToNodeIdx(numStart + (1:numGoal));

costMatrix = inf(numNodes, numNodes);
pathCache = cell(numNodes, numNodes);
for i = 1:numNodes
    costMatrix(i, i) = 0;
    pathCache{i, i} = nodes(i, :);
end

roadmap = [];
obstacles = {};
if isfield(plannerOpts, 'plannerType') && strcmpi(plannerOpts.plannerType, 'prm')
    stayAwayPoints = localGetField(plannerOpts, 'stayAwayPoints', zeros(0, 2));
    robotInflation = localGetField(plannerOpts, 'robotInflation', 0.22);
    nPRM = localGetField(plannerOpts, 'nPRM', 150);
    obstacles = buildObstacles(stayAwayPoints);
    goalForSampling = nodes(min(2, numNodes), :);
    roadmap = buildPRM(obstacles, navMap, plannerOpts.boundary, nodes(1, :), ...
        goalForSampling, nPRM, robotInflation, stayAwayPoints, @haltonSample);
end

for i = 1:numNodes
    for j = i + 1:numNodes
        [path, pathLen] = localPlanPath(navMap, nodes(i, :), nodes(j, :), plannerOpts, roadmap, obstacles);
        if isempty(path)
            continue;
        end
        costMatrix(i, j) = pathLen;
        costMatrix(j, i) = pathLen;
        pathCache{i, j} = path;
        pathCache{j, i} = flipud(path);
    end
end

nodeMeta = struct();
nodeMeta.candidateStartNodeIdx = candidateStartNodeIdx;
nodeMeta.goalNodeIdx = goalNodeIdx;
nodeMeta.isCandidateStart = false(numNodes, 1);
nodeMeta.isGoal = false(numNodes, 1);
nodeMeta.startIndices = cell(numNodes, 1);
nodeMeta.goalIndices = cell(numNodes, 1);

for i = 1:numNodes
    startMatches = find(candidateStartNodeIdx == i);
    goalMatches = find(goalNodeIdx == i);
    nodeMeta.isCandidateStart(i) = ~isempty(startMatches);
    nodeMeta.isGoal(i) = ~isempty(goalMatches);
    nodeMeta.startIndices{i} = startMatches(:)';
    nodeMeta.goalIndices{i} = goalMatches(:)';
end
end

function [path, pathLen] = localPlanPath(navMap, startXY, goalXY, plannerOpts, roadmap, obstacles)
path = zeros(0, 2);
pathLen = inf;

plannerType = localGetField(plannerOpts, 'plannerType', 'astar');

if strcmpi(plannerType, 'prm')
    stayAwayPoints = localGetField(plannerOpts, 'stayAwayPoints', zeros(0, 2));
    robotInflation = localGetField(plannerOpts, 'robotInflation', 0.22);
    path = findPath(roadmap, startXY, goalXY, obstacles, navMap, stayAwayPoints, robotInflation);
else
    astarOpts = struct( ...
        'resolution', localGetField(plannerOpts, 'planResolution', 0.10), ...
        'inflationRadius', localGetField(plannerOpts, 'robotInflation', 0.22), ...
        'stayAwayPoints', localGetField(plannerOpts, 'stayAwayPoints', zeros(0, 2)), ...
        'stayAwayRadius', localGetField(plannerOpts, 'stayAwayInflation', 0.25), ...
        'preferredClearance', localGetField(plannerOpts, 'preferredClearance', 0.42), ...
        'clearanceWeight', localGetField(plannerOpts, 'clearanceWeight', 1.5), ...
        'maxShortcutLength', localGetField(plannerOpts, 'maxShortcutLength', 0.25));
    [path, ~, ~, found] = planPathAStar(navMap, plannerOpts.boundary, startXY, goalXY, astarOpts);
    if ~found
        path = zeros(0, 2);
    end
end

if isempty(path)
    return;
end

pathLen = sum(vecnorm(diff(path), 2, 2));
end

function value = localGetField(s, name, defaultValue)
if isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end
