% Pursuit Guidance
clearvars; % fresh workspace
close all; % close figures
clc; % clear console

K = 1.5; % a pursuer-to-target speed ratio greater than one
vT = 1.0; % the target speed
pursuerInitial = [0, 0]; % set initial pursuer position
targetInitial = [60, 50]; % set initial target position
[r0, lambda0] = computeInitialLOS(pursuerInitial(1), pursuerInitial(2), targetInitial(1), targetInitial(2)); % initial LOS data
theta0 = 2 * pi / 3; % set initial aspect angle requested in Exercise 1
gammaT = lambda0 + theta0; % the target heading so the simulated initial aspect angle equals theta0
ratio = purePursuitLengthRatio(K, theta0, pi / 3); % requested pure-pursuit path-length ratio

deltaNominal = deg2rad(10); % a valid nominal lead angle with abs(K*sin(delta)) less than one
deltaBoundary = asin(sin(theta0) / K); % the special case K*sin(delta*) = sin(theta0)
deltaInvalid = asin(1 / K) + deg2rad(7); % a case with abs(K*sin(delta*)) greater than one
tFinal = 220; % integration horizon
hitRadius = 0.25; % intercept tolerance

nominalResult = simulateDeviatedPurePursuit(pursuerInitial, targetInitial, K, vT, gammaT, deltaNominal, tFinal, hitRadius); % simulate a normal valid deviated-pursuit case
boundaryResult = simulateDeviatedPurePursuit(pursuerInitial, targetInitial, K, vT, gammaT, deltaBoundary, tFinal, hitRadius); % simulate the K*sin(delta*) = sin(theta0) case
invalidResult = simulateDeviatedPurePursuit(pursuerInitial, targetInitial, K, vT, gammaT, deltaInvalid, tFinal, hitRadius); % simulate the abs(K*sin(delta*)) greater than one case

fprintf("\nInitial LOS range r0 = %.4f\n", r0); % initial LOS range
fprintf("Initial LOS angle lambda0 = %.4f rad\n", lambda0); % initial LOS angle
fprintf("Exercise 1 length ratio l(pi/3)/l(0) for K = %.2f is %.6f\n\n", K, ratio); % the ratio for the chosen K

caseName = ["Valid delta"; "Boundary delta"; "Invalid delta"]; % case labels
deltaRad = [deltaNominal; deltaBoundary; deltaInvalid]; % lead angles in radians
KSinDelta = K * sin(deltaRad); % key K*sin(delta) quantity
intercepted = [nominalResult.intercepted; boundaryResult.intercepted; invalidResult.intercepted]; % intercept flags
finalRange = [nominalResult.finalRange; boundaryResult.finalRange; invalidResult.finalRange]; % final ranges
eventTime = [firstOrNaN(nominalResult.eventTime); firstOrNaN(boundaryResult.eventTime); firstOrNaN(invalidResult.eventTime)]; % intercept times
summaryTable = table(caseName, deltaRad, KSinDelta, intercepted, eventTime, finalRange); % simulation summary table
disp(summaryTable); % the simulation summary table

figure("Color", "w", "Name", "Pursuit Guidance Trajectories"); % trajectory figure
tiledlayout(1, 3, "Padding", "compact", "TileSpacing", "compact"); % one tile for each requested behavior case
plotTrajectoryCase(nominalResult, "Valid case"); % the nominal valid case
plotTrajectoryCase(boundaryResult, "Boundary case"); % the boundary case
plotTrajectoryCase(invalidResult, "Invalid-formula case"); % the invalid case
saveas(gcf, fullfile(fileparts(mfilename("fullpath")), "deviated_pursuit_trajectories.png")); % save the trajectory figure

figure("Color", "w", "Name", "Pursuit Guidance Histories"); % history figure
tiledlayout(2, 1, "Padding", "compact", "TileSpacing", "compact"); % range and aspect-angle tiles
nexttile; % range-history tile
plot(nominalResult.time, nominalResult.range, "LineWidth", 1.6); % nominal range history
hold on; % the range axes for additional cases
plot(boundaryResult.time, boundaryResult.range, "LineWidth", 1.6); % boundary range history
plot(invalidResult.time, invalidResult.range, "LineWidth", 1.6); % invalid range history
grid on; % grid to the range plot
xlabel("time"); % time axis
ylabel("range r"); % range axis
legend("Valid", "Boundary", "Invalid", "Location", "best"); % range-history legend
title("Line-of-sight range"); % range-history tile
nexttile; % aspect-angle history tile
plot(nominalResult.time, nominalResult.theta, "LineWidth", 1.6); % nominal aspect-angle history
hold on; % the aspect axes for additional cases
plot(boundaryResult.time, boundaryResult.theta, "LineWidth", 1.6); % boundary aspect-angle history
plot(invalidResult.time, invalidResult.theta, "LineWidth", 1.6); % invalid aspect-angle history
grid on; % grid to the aspect-angle plot
xlabel("time"); % time axis
ylabel("aspect angle theta"); % aspect-angle axis
legend("Valid", "Boundary", "Invalid", "Location", "best"); % aspect-angle legend
title("Aspect angle evolution"); % aspect-angle tile
saveas(gcf, fullfile(fileparts(mfilename("fullpath")), "range_aspect_histories.png")); % save the range/aspect history figure

if ~usejava("desktop") % MATLAB is running in batch batch mode
    close all; % figures so batch execution exits cleanly
end % batch cleanup

function value = firstOrNaN(vectorValue) % empty event-time vector to NaN
if isempty(vectorValue) % no event time was recorded
    value = NaN; % naN when there was no intercept
else % a nonempty event-time vector
    value = vectorValue(1); % first recorded event time
end % empty vector branch
end % firstOrNaN

function plotTrajectoryCase(result, titleText) % one deviated-pursuit trajectory case
nexttile; % advance to the next tiled-layout axes
plot(result.target(:, 1), result.target(:, 2), "k--", "LineWidth", 1.2); % target trajectory
hold on; % axes for pursuer and markers
plot(result.pursuer(:, 1), result.pursuer(:, 2), "b-", "LineWidth", 1.8); % pursuer trajectory
plot(result.pursuer(1, 1), result.pursuer(1, 2), "go", "MarkerFaceColor", "g", "MarkerSize", 6); % pursuer start
plot(result.target(1, 1), result.target(1, 2), "ko", "MarkerFaceColor", "k", "MarkerSize", 6); % target start
plot(result.target(end, 1), result.target(end, 2), "mo", "MarkerFaceColor", "m", "MarkerSize", 6); % target final or intercept point
axis equal; % equal scaling for geometric interpretation
grid on; % grid lines
xlabel("x"); % x axis
ylabel("y"); % y axis
title(titleText); % case title
legend("Target", "Pursuer", "Pursuer start", "Target start", "Final target", "Location", "best"); % compact legend
end % plotTrajectoryCase

