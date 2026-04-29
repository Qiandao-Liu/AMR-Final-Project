function dataStore = readAllSensors(Robot, dataStore)

    t = toc;
    
    %% --- ODOMETRY ---
    try
        d = DistanceSensorRoomba(Robot);
        a = AngleSensorRoomba(Robot);
        dataStore.odometry = [dataStore.odometry; t, d, a];
    catch
    end
    
    %% --- DEPTH ---
    try
        depth = RealSenseDist(Robot);
        dataStore.rsdepth = [dataStore.rsdepth; t, depth'];
    catch
    end
    
    %% --- BUMP ---
    try
        [BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront] = ...
            BumpsWheelDropsSensorsRoomba(Robot);
    
        dataStore.bump = [dataStore.bump; t, ...
            BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront];
    catch
    end
    
    %% --- BEACON ---
    try
        tags = RealSenseTag(Robot);
    
        if ~isempty(tags)
            % Each row: [time, tagID, x, y, ...]
            dataStore.beacon = [dataStore.beacon; ...
                repmat(t, size(tags,1),1), tags];
        end
    catch
    end

end