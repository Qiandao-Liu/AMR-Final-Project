function obstacles = buildObstacles(stayAwayPoints)

    obs_radius = 0.05;
    obstacles = {};
    
    for i = 1:size(stayAwayPoints,1)
        x = stayAwayPoints(i,1);
        y = stayAwayPoints(i,2);
    
        obstacles{end+1} = [
            x-obs_radius y-obs_radius;
            x+obs_radius y-obs_radius;
            x+obs_radius y+obs_radius;
            x-obs_radius y+obs_radius
        ];
    end
end