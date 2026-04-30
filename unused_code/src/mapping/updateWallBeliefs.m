function wallBeliefs = updateWallBeliefs(robotPose, zDepth, knownMap, optWalls, wallBeliefs, sensorOrigin, angles)
% UPDATEWALLBELIEFS Update the existence probability of optional walls using Bayesian logic.
%
%   wallBeliefs = updateWallBeliefs(robotPose, zDepth, knownMap, optWalls, ...
%                                   wallBeliefs, sensorOrigin, angles)
%
%   INPUTS
%       robotPose    3x1 current best pose estimate [x; y; theta]
%       zDepth       Kx1 actual depth measurements from sensor
%       knownMap     Nx4 static wall matrix (permanent walls)
%       optWalls     Mx4 optional wall matrix
%       wallBeliefs  Mx1 struct array (from initWallBeliefs)
%       sensorOrigin 2x1 sensor origin in robot frame
%       angles       Kx1 beam angles in sensor frame
%
%   OUTPUT
%       wallBeliefs  Updated struct array with new probabilities and statuses

% --- Parameters (Can be moved to a config struct later) ---
maxReliableRange = 4.0;      % Competition spec
sigma = 0.15;                % Standard deviation of depth noise (m)
probThresholdPresent = 0.90; % Threshold to declare wall as 'present'
probThresholdAbsent = 0.10;  % Threshold to declare wall as 'absent'
minProb = 0.01;              % Clamping to avoid 0/1 probability lock
maxProb = 0.99;

numBeams = length(zDepth);
numOptWalls = size(optWalls, 1);

% 1. Predict what we SHOULD see for both Static and Optional maps.
[dStatic, ~] = depthPredictNew(robotPose, knownMap, sensorOrigin, angles, 10.0);
[dOpt, hitIdxOpt] = depthPredictNew(robotPose, optWalls, sensorOrigin, angles, 10.0);

% 2. Process each beam
for k = 1:numBeams
    wallIdx = hitIdxOpt(k);
    
    % Only proceed if this beam hit an optional wall
    if wallIdx == 0
        continue;
    end
    
    % --- Occlusion Check ---
    % If a permanent wall is closer than the optional wall, we can't see the optWall.
    if dStatic(k) < dOpt(k) - 0.05
        continue;
    end
    
    % --- Range Check ---
    % Only update if the optional wall is within the reliable sensor range.
    if dOpt(k) > maxReliableRange
        continue;
    end
    
    % Current measurement and predicted distance to the optWall
    z = zDepth(k);
    d_pred = dOpt(k);
    
    % 3. Bayesian Update Logic
    % Prior probability
    P_old = wallBeliefs(wallIdx).probPresent;
    
    % Likelihood P(z | Wall is Present): Normal distribution around predicted wall
    L_present = normpdf(z, d_pred, sigma);
    
    % Likelihood P(z | Wall is Absent): 
    % If wall is absent, the ray should hit the background (Static wall or max range)
    L_absent = normpdf(z, dStatic(k), sigma);
    
    % Edge case: If z is much larger than d_pred but not hitting dStatic (e.g., hitting nothing)
    % we still want to count it as evidence for "Absent".
    if z > d_pred + 3*sigma
        L_absent = max(L_absent, normpdf(d_pred + 3*sigma, d_pred, sigma)); 
    end
    
    % Bayesian Posterior formula:
    % P(H|z) = [P(z|H) * P(H)] / [P(z|H)*P(H) + P(z|notH)*P(notH)]
    numerator = L_present * P_old;
    denominator = L_present * P_old + L_absent * (1 - P_old);
    
    if denominator > 0
        P_new = numerator / denominator;
        % Clamp for numerical stability
        P_new = max(min(P_new, maxProb), minProb);
        
        % Update struct
        wallBeliefs(wallIdx).probPresent = P_new;
        wallBeliefs(wallIdx).observationCount = wallBeliefs(wallIdx).observationCount + 1;
    end
end

% 4. Final Status Update
% Update the string status based on thresholds
for i = 1:numOptWalls
    p = wallBeliefs(i).probPresent;
    if p > probThresholdPresent
        wallBeliefs(i).status = 'present';
    elseif p < probThresholdAbsent
        wallBeliefs(i).status = 'absent';
    else
        wallBeliefs(i).status = 'unknown';
    end
end

end
