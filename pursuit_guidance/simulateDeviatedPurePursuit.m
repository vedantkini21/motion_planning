% Pursuit Guidance
function result = simulateDeviatedPurePursuit(pursuerInitial, targetInitial, K, vT, gammaT, deltaStar, tFinal, hitRadius) % simulate deviated pure pursuit with ode45
vM = K * vT; % pursuer speed from the speed ratio
initialState = [pursuerInitial(:); targetInitial(:)]; % oDE state vector [xM;yM;xT;yT]
options = odeset("RelTol", 1e-8, "AbsTol", 1e-10, "Events", @(time, state) interceptEvent(time, state, hitRadius)); % accurate integration and intercept stopping
[time, state, eventTime, eventState] = ode45(@(time, state) pursuitDynamics(time, state, vM, vT, gammaT, deltaStar), [0, tFinal], initialState, options); % the relative motion
relativeX = state(:, 3) - state(:, 1); % target-minus-pursuer x coords over time
relativeY = state(:, 4) - state(:, 2); % target-minus-pursuer y coords over time
range = sqrt(relativeX.^2 + relativeY.^2); % lOS range over time
lambda = atan2(relativeY, relativeX); % lOS angle over time
theta = wrapAngle(gammaT - lambda); % aspect angle relative to the target velocity direction
result.time = time; % simulated time samples
result.state = state; % full state history
result.pursuer = state(:, 1:2); % pursuer coords
result.target = state(:, 3:4); % target coords
result.range = range; % lOS range history
result.lambda = lambda; % lOS angle history
result.theta = theta; % aspect-angle history
result.deltaStar = deltaStar; % commanded lead angle
result.K = K; % speed ratio
result.vT = vT; % target speed
result.vM = vM; % pursuer speed
result.gammaT = gammaT; % target flight-path angle
result.intercepted = ~isempty(eventTime); % whether the intercept event occurred
result.eventTime = eventTime; % intercept event time if present
result.eventState = eventState; % intercept event state if present
result.finalRange = range(end); % final simulated range
end % simulateDeviatedPurePursuit

function derivative = pursuitDynamics(~, state, vM, vT, gammaT, deltaStar) % four-state kinematic model
xM = state(1); % pursuer x position
yM = state(2); % pursuer y position
xT = state(3); % target x position
yT = state(4); % target y position
lambda = atan2(yT - yM, xT - xM); % current LOS angle from pursuer to target
gammaM = lambda + deltaStar; % the deviated pure-pursuit law with constant lead angle
xMDot = vM * cos(gammaM); % pursuer x velocity
yMDot = vM * sin(gammaM); % pursuer y velocity
xTDot = vT * cos(gammaT); % target x velocity
yTDot = vT * sin(gammaT); % target y velocity
derivative = [xMDot; yMDot; xTDot; yTDot]; % oDE derivative vector
end % pursuitDynamics

function [value, isterminal, direction] = interceptEvent(~, state, hitRadius) % ode45 event for target interception
range = sqrt((state(3) - state(1))^2 + (state(4) - state(2))^2); % current LOS range
value = range - hitRadius; % the event when range reaches the hit radius
isterminal = 1; % integration at interception
direction = -1; % detect only decreasing range crossings
end % interceptEvent

function wrapped = wrapAngle(angleValue) % wrap angles into the interval [-pi, pi]
wrapped = mod(angleValue + pi, 2 * pi) - pi; % a toolbox-free angle wrap
end % wrapAngle

