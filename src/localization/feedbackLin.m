function [cmdV, cmdW] = feedbackLin(cmdVx, cmdVy, theta, epsilon)
% FEEDBACKLIN Convert desired planar velocity to differential-drive commands.

cmdV = cos(theta) * cmdVx + sin(theta) * cmdVy;
cmdW = (-sin(theta) * cmdVx + cos(theta) * cmdVy) / epsilon;
end
