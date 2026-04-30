function valid = collisionFree(p1, p2, obstacles, walls, stayAwayPoints, epsilon_wall, epsilon_stay)

    dist = norm(p2 - p1);
    n = max(10, ceil(dist/0.05));

    for i = 1:n

        p = (1-i/n)*p1 + (i/n)*p2;

        % polygon obstacles
        for k = 1:length(obstacles)
            poly = obstacles{k};

            if inpolygon(p(1),p(2),poly(:,1),poly(:,2))
                valid = false;
                return;
            end
        end

        % wall clearance (RELAXED)
        for k = 1:size(walls,1)
            if pointToSegmentDist(p, walls(k,1:2), walls(k,3:4)) < epsilon_wall
                valid = false;
                return;
            end
        end
    end

    % stay-away (STRICT)
    for k = 1:size(stayAwayPoints,1)
        if edgeTooCloseToPoint(p1,p2,stayAwayPoints(k,:),epsilon_stay)
            valid = false;
            return;
        end
    end

    valid = true;
end