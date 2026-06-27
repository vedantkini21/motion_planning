% Pursuit Guidance
function ratio = purePursuitLengthRatio(K, theta0, thetaQuery) % l(thetaQuery)/l(0) for pure pursuit from theta0 to intercept
if K <= 1 % the speed-ratio assumption
    error("K must be greater than 1 for the closed-form pure-pursuit length ratio."); % invalid speed ratio
end % speed-ratio check
totalRemainingAtLaunch = remainingPurePursuitLengthFactor(K, theta0); % total path-length factor from launch to intercept
remainingAtQuery = remainingPurePursuitLengthFactor(K, thetaQuery); % remaining path-length factor at the queried angle
ratio = (totalRemainingAtLaunch - remainingAtQuery) / totalRemainingAtLaunch; % remaining lengths into traveled-length ratio
end % purePursuitLengthRatio

function value = remainingPurePursuitLengthFactor(K, thetaValue) % common remaining-length factor without the range constant
tangentHalfAngle = tan(thetaValue / 2); % transform the aspect angle using t = tan(theta/2)
value = tangentHalfAngle^(K - 1) / (K - 1) + tangentHalfAngle^(K + 1) / (K + 1); % evaluate the closed-form integral factor
end % remainingPurePursuitLengthFactor

