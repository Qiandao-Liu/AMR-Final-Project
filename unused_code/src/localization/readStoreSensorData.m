function [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore)
% READSTORESENSORDATA Read simulator/robot sensors into a log struct.

try
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

try
    try
        [px, py, pt] = OverheadLocalizationCreate(Robot);
        dataStore.truthPose = [dataStore.truthPose; toc, px, py, pt];
        noRobotCount = 0;
    catch
        noRobotCount = noRobotCount + 1;
    end
catch
    disp('Error retrieving or saving overhead localization data.');
end

try
    deltaD = DistanceSensorRoomba(CreatePort);
    deltaA = AngleSensorRoomba(CreatePort);
    dataStore.odometry = [dataStore.odometry; toc, deltaD, deltaA];
catch
    disp('Error retrieving or saving odometry data.');
end

try
    [BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront] = ...
        BumpsWheelDropsSensorsRoomba(CreatePort);
    dataStore.bump = [dataStore.bump; toc, ...
        BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront];
catch
    disp('Error retrieving or saving bump sensor data.');
end

try
    depthArray = RealSenseDist(Robot);
    dataStore.rsdepth = [dataStore.rsdepth; toc, depthArray'];
catch
    disp('Error retrieving or saving RealSense depth data.');
end

try
    tags = RealSenseTag(Robot);
    if ~isempty(tags)
        dataStore.beacon = [dataStore.beacon; repmat(toc, size(tags, 1), 1), tags];
    end
catch
    disp('Error retrieving or saving beacon data.');
end
end
