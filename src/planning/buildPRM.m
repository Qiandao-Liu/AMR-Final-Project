function PRM = buildPRM(obstacles, walls, boundary, q_start, q_goal, n_PRM, r, stayAway, sampleFunc)
% BUILDPRM 

    boundary = boundary(:)';

    % ---------------------------
    % Initialize vertices
    % ---------------------------
    q_start = q_start(:)';
    q_goal  = q_goal(:)';

    V = [q_start; q_goal];
    i = 1;

    goalBias = 0.1;
    k = 15;

    % ---------------------------
    % Sampling loop
    % ---------------------------
    while size(V,1) < n_PRM + 2

        if rand < goalBias
            q = q_goal;
        else
            q = sampleFunc(i, boundary);
        end

        i = i + 1;

        if isFree(q, obstacles, walls, boundary, r)
            V = [V; q];
        end
    end

    % ---------------------------
    % Build edges (k-NN graph)
    % ---------------------------
    E = [];
    N = size(V,1);

    for i = 1:N

        d = vecnorm(V - V(i,:), 2, 2);
        [~, idx] = sort(d);

        neighbors = idx(2:min(k+1,N));

        for j = neighbors'

            if collisionFree(V(i,:), V(j,:), obstacles, walls, stayAway, r, 1.5*r)
                E = [E; i j];
            end
        end
    end

    % ---------------------------
    % Output
    % ---------------------------
    PRM.V = V;
    PRM.E = E;
    PRM.startIdx = 1;
    PRM.goalIdx = 2;

end








function free = isFree(q, obstacles, walls, boundary, epsilon_wall)

    xmin = boundary(1);
    ymin = boundary(2);
    xmax = boundary(3);
    ymax = boundary(4);

    % boundary check
    if q(1) < xmin || q(1) > xmax || q(2) < ymin || q(2) > ymax
        free = false;
        return;
    end

    % polygon obstacles
    for i = 1:length(obstacles)
        poly = obstacles{i};

        if inpolygon(q(1), q(2), poly(:,1), poly(:,2))
            free = false;
            return;
        end
    end

    % line walls
    for i = 1:size(walls,1)

        a = walls(i,1:2);
        b = walls(i,3:4);

        if pointToSegmentDist(q,a,b) < epsilon_wall
            free = false;
            return;
        end
    end

    free = true;
end





