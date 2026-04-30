function path = findPath(roadmap, start, goal, obstacles, walls, stayAwayPoints, epsilon)
    
    nodes = roadmap.V;
    edges = roadmap.E;
    
    start = start(:)';
    goal  = goal(:)';
    
    nodes = [nodes; start; goal];
    
    s_idx = size(nodes,1)-1;
    g_idx = size(nodes,1);
    
    % ---------------------------
    % connect start/goal
    % ---------------------------
    for i = 1:size(nodes,1)-2
    
        if isVisible(nodes(i,:), start, obstacles, walls, stayAwayPoints, epsilon)
            edges = [edges; s_idx i];
        end
    
        if isVisible(nodes(i,:), goal, obstacles, walls, stayAwayPoints, epsilon)
            edges = [edges; g_idx i];
        end
    end
    
    % ---------------------------
    % graph build
    % ---------------------------
    s = [];
    t = [];
    w = [];
    
    for i = 1:size(edges,1)
    
        a = edges(i,1);
        b = edges(i,2);
    
        s = [s; a];
        t = [t; b];
        w = [w; norm(nodes(a,:) - nodes(b,:))];
    end
    
    G = graph(s,t,w,size(nodes,1));
    
    [path_idx, ~] = shortestpath(G, s_idx, g_idx, 'Method','positive');
    
    path = nodes(path_idx,:);
end
