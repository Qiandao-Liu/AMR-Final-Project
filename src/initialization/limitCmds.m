function [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center)
% LIMITCMDS Scale differential-drive commands to avoid wheel saturation.
%
%   [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center)

vLeft = fwdVel - angVel * wheel2Center;
vRight = fwdVel + angVel * wheel2Center;

maxWheelVel = max(abs([vLeft, vRight]));
if maxWheelVel > maxV
    scaleFactor = maxV / maxWheelVel;
    vLeft = vLeft * scaleFactor;
    vRight = vRight * scaleFactor;
end

cmdV = (vRight + vLeft) / 2;
cmdW = (vRight - vLeft) / (2 * wheel2Center);
end
