function isPresent = verifyWallPresence(zDepth)
% VERIFYWALLPRESENCE Confirm if an optional wall is present at the probe pose.
%
%   isPresent = verifyWallPresence(zDepth)
%
%   INPUTS
%       zDepth       Kx1 vector of actual depth measurements.
%
%   OUTPUT
%       isPresent    Logical: true if the wall is detected, false otherwise.
%
%   Note: This function assumes the robot is positioned exactly 0.5m from 
%         the wall center and is facing it directly. 
%         Expected sensor depth = 0.5 - 0.08 = 0.42m.

% --- Parameters ---
expectedDist = 0.42;    % (probeDist - sensorOffsetX)
tolerance = 0.15;       % Allow for +/- 15cm error (noise + pose inaccuracy)
numBeamsToAverage = 5;  % Number of central beams to inspect

% 1. Identify the central beams
numBeams = length(zDepth);
midIdx = floor(numBeams / 2) + 1;
halfWindow = floor(numBeamsToAverage / 2);

% Extract a window around the center (0 degrees)
centerBeams = zDepth(midIdx - halfWindow : midIdx + halfWindow);

% 2. Robust Average
% Use median to reject potential outliers (e.g., if the wall is very short)
observedDist = median(centerBeams);

% 3. Decision Logic
% We are looking for a reading that is consistently near 0.42m.
% If the wall is absent, the sensor will read the background (> 1.0m) 
% or max range (3.0m).
if observedDist >= (expectedDist - tolerance) && ...
   observedDist <= (expectedDist + tolerance)
    
    isPresent = true;
    fprintf('Mapping: Wall confirmed PRESENT (Observed: %.2fm, Expected: %.2fm)\n', ...
        observedDist, expectedDist);
else
    isPresent = false;
    fprintf('Mapping: Wall confirmed ABSENT (Observed: %.2fm, Expected: %.2fm)\n', ...
        observedDist, expectedDist);
end

end