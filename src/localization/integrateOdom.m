function finalPose = integrateOdom(initPose, d, phi)
% INTEGRATEODOM Propagate a differential-drive pose using odometry.
%
%   finalPose = integrateOdom(initPose, d, phi)
%
%   INPUTS
%       initPose    3x1 initial pose [x; y; theta]
%       d           scalar or 1xN forward distance
%       phi         scalar or 1xN heading change
%
%   OUTPUT
%       finalPose   3xN pose trajectory, last column is the latest pose

N = length(d);
finalPose = zeros(3, N);
currentPose = initPose;

for i = 1:N
    x = currentPose(1);
    y = currentPose(2);
    theta = currentPose(3);

    dStep = d(i);
    phiStep = phi(i);

    if abs(phiStep) < 1e-6
        xNew = x + dStep * cos(theta);
        yNew = y + dStep * sin(theta);
        thetaNew = theta;
    else
        radius = dStep / phiStep;
        xNew = x + radius * (sin(theta + phiStep) - sin(theta));
        yNew = y - radius * (cos(theta + phiStep) - cos(theta));
        thetaNew = theta + phiStep;
    end

    currentPose = [xNew; yNew; thetaNew];
    finalPose(:, i) = currentPose;
end
end
