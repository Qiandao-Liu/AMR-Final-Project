function vis = isVisible(p1, p2, obstacles, walls, stayAwayPoints, epsilon)

vis = true;

n = max(10, ceil(norm(p2-p1)/0.05));

for i = 1:n

    p = (1-i/n)*p1 + (i/n)*p2;

    % -----------------------
    % polygon obstacles
    % -----------------------
    for k = 1:length(obstacles)

        poly = obstacles{k};

        if inpolygon(p(1),p(2),poly(:,1),poly(:,2))
            vis = false;
            return;
        end
    end

    % -----------------------
    % wall collision check (CRITICAL FIX)
    % -----------------------
    for k = 1:size(walls,1)

        a = walls(k,1:2);
        b = walls(k,3:4);

        % distance from sampled point to wall segment
        if pointToSegmentDist(p,a,b) < epsilon
            vis = false;
            return;
        end
    end

    % -----------------------
    % stay-away clearance
    % -----------------------
    for k = 1:size(stayAwayPoints,1)

        %if norm(p - stayAwayPoints(k,:)) < epsilon
        if edgeTooCloseToPoint(p1, p2, stayAwayPoints(k,:), epsilon)
            vis = false;
            return;
        end
    end
end
end