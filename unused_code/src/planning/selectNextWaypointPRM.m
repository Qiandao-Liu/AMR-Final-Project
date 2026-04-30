function [bestWP, bestIdx] = selectNextWaypointPRM(points, q, PRM, obstacles, walls, stayAwayPoints, robotRadius)

    bestCost = inf;
    bestIdx = 1;
    
    for i = 1:size(points,1)
    
        path = findPath(PRM, q, points(i,:), ...
                        obstacles, walls, stayAwayPoints, robotRadius);
    
        if isempty(path)
            continue;
        end
    
        cost = sum(vecnorm(diff(path),2,2));
    
        if cost < bestCost
            bestCost = cost;
            bestIdx = i;
        end
    end
    
    bestWP = points(bestIdx,:);
end