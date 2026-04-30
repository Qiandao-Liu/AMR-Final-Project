function [shouldReplan, blockedSegmentIdx] = replanIfBlockedByConfirmedWall(confirmedWall, currentPath, remainingVisitNodeOrder, pathCache, clearance)
% REPLANIFBLOCKEDBYCONFIRMEDWALL Return true when a newly confirmed wall
% intersects the current local path or any cached future route segment.

if nargin < 5
    clearance = 0.0;
end

shouldReplan = false;
blockedSegmentIdx = [];

walls = confirmedWall;
if isempty(walls)
    return;
end

for w = 1:size(walls, 1)
    if localPathBlocked(currentPath, walls(w, :), clearance)
        shouldReplan = true;
        blockedSegmentIdx = 0;
        return;
    end

    for i = 1:max(numel(remainingVisitNodeOrder) - 1, 0)
        path = pathCache{remainingVisitNodeOrder(i), remainingVisitNodeOrder(i + 1)};
        if localPathBlocked(path, walls(w, :), clearance)
            shouldReplan = true;
            blockedSegmentIdx = i;
            return;
        end
    end
end
end

function blocked = localPathBlocked(path, wall, clearance)
blocked = false;
if isempty(path) || size(path, 1) < 2
    return;
end

w1 = wall(1:2);
w2 = wall(3:4);

for i = 1:size(path, 1) - 1
    p1 = path(i, :);
    p2 = path(i + 1, :);
    if segmentToSegmentDist(p1, p2, w1, w2) <= clearance
        blocked = true;
        return;
    end
end
end
