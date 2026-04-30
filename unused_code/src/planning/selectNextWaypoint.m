function [bestWaypoint, bestIdx] = selectNextWaypoint(points, q_current, PRM, obstacles, walls, stayAwayPoints, robotRadius)

    n = size(points,1);
    bestCost = inf;
    bestIdx = 1;

    for i = 1:n
        path = findPath(PRM, q_current, points(i,:), ...
                        obstacles, walls, stayAwayPoints, robotRadius);

        if ~isempty(path)
            diffs = diff(path);
            cost = sum(vecnorm(diffs,2,2));

            if cost < bestCost
                bestCost = cost;
                bestIdx = i;
            end
        end
    end

    bestWaypoint = points(bestIdx,:);
end