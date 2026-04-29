function [pathIdx, found] = astarGrid(occupancy, startIdx, goalIdx, traversalCost)
% ASTARGRID Run A* on a binary occupancy grid using 8-connected motion.
%
%   [pathIdx, found] = astarGrid(occupancy, startIdx, goalIdx)

[ny, nx] = size(occupancy);
if nargin < 4 || isempty(traversalCost)
    traversalCost = zeros(ny, nx);
end
startLin = sub2ind([ny, nx], startIdx(1), startIdx(2));
goalLin = sub2ind([ny, nx], goalIdx(1), goalIdx(2));

if occupancy(startLin) || occupancy(goalLin)
    pathIdx = zeros(0, 2);
    found = false;
    return;
end

gScore = inf(ny, nx);
fScore = inf(ny, nx);
cameFrom = zeros(ny, nx, 2);
openSet = false(ny, nx);
closedSet = false(ny, nx);

gScore(startLin) = 0;
fScore(startLin) = heuristic(startIdx, goalIdx);
openSet(startLin) = true;

neighbors = [ ...
    -1, -1; -1, 0; -1, 1; ...
     0, -1;          0, 1; ...
     1, -1;  1, 0;  1, 1];

while any(openSet(:))
    openF = fScore;
    openF(~openSet) = inf;
    [~, currentLin] = min(openF(:));
    [cy, cx] = ind2sub([ny, nx], currentLin);
    current = [cy, cx];

    if currentLin == goalLin
        pathIdx = reconstructPath(cameFrom, current);
        found = true;
        return;
    end

    openSet(currentLin) = false;
    closedSet(currentLin) = true;

    for k = 1:size(neighbors, 1)
        neighbor = current + neighbors(k, :);
        nyi = neighbor(1);
        nxi = neighbor(2);

        if nyi < 1 || nyi > ny || nxi < 1 || nxi > nx
            continue;
        end

        if occupancy(nyi, nxi) || closedSet(nyi, nxi)
            continue;
        end

        stepCost = norm(neighbors(k, :)) + traversalCost(nyi, nxi);
        tentativeG = gScore(cy, cx) + stepCost;

        if ~openSet(nyi, nxi)
            openSet(nyi, nxi) = true;
        elseif tentativeG >= gScore(nyi, nxi)
            continue;
        end

        cameFrom(nyi, nxi, :) = current;
        gScore(nyi, nxi) = tentativeG;
        fScore(nyi, nxi) = tentativeG + heuristic(neighbor, goalIdx);
    end
end

pathIdx = zeros(0, 2);
found = false;
end

function h = heuristic(a, b)
h = norm(double(a) - double(b));
end

function pathIdx = reconstructPath(cameFrom, current)
pathIdx = current;

while true
    prev = squeeze(cameFrom(current(1), current(2), :))';
    if all(prev == 0)
        break;
    end
    current = prev;
    pathIdx = [current; pathIdx]; %#ok<AGROW>
end
end
