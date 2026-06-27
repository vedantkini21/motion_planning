% Pursuit Guidance
function [r0, lambda0] = computeInitialLOS(xM0, yM0, xT0, yT0) % initial LOS range and angle
dx = xT0 - xM0; % initial relative x coord from pursuer to target
dy = yT0 - yM0; % initial relative y coord from pursuer to target
r0 = sqrt(dx^2 + dy^2); % euclidean LOS range
lambda0 = atan2(dy, dx); % lOS angle measured from the positive x axis
end % computeInitialLOS

