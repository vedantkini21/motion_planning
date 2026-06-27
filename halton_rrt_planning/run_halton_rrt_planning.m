% Halton RRT Planning
clearvars; % fresh workspace
close all; % close figures
clc; % clear console

rng(12); % reproducible run
startPosition = [8, 10]; % robot initial centroid position
goalPosition = [92, 88]; % robot goal centroid position
obstacles = createRandomNonConvexObstacles(); % five deterministic randomly shifted non-convex obstacles
haltonPoints = computeGridHalton(100, 2, 3); % 100 Halton samples using the first two prime bases

figure("Color", "w", "Name", "Halton RRT Planning Samples"); % halton sample figure
plot(haltonPoints(:, 1), haltonPoints(:, 2), "k.", "MarkerSize", 12); % the 100 Halton points in the workspace
axis([0, 100, 0, 100]); % the axis limits to the assignment workspace
axis square; % equal x and y scaling
grid on; % show a light grid for sample distribution
xlabel("x"); % x axis
ylabel("y"); % y axis
title("100 Halton samples in [0,100] x [0,100]"); % descriptive title
saveas(gcf, fullfile(fileparts(mfilename("fullpath")), "halton_samples.png")); % save the Halton sample figure

rng(4); % RRT seed
[RRTPath, RRTTree, RRTStats] = RRTSearch(startPosition, goalPosition, obstacles); % required RRT search algorithm
rng(4); % RRT-star seed
[starPath, starTree, starStats] = RRTStarSearch(startPosition, goalPosition, obstacles); % bonus shortest-path RRT-star algorithm

fprintf("\nCollision detection test, separated polygons: %d\n", collisionDetection([0,0; 2,0; 1,1], [5,5; 7,5; 6,6])); % a no-collision test
fprintf("Collision detection test, overlapping polygons: %d\n\n", collisionDetection([0,0; 3,0; 3,3; 0,3], [2,2; 5,2; 5,5; 2,5])); % a collision test

algorithm = ["RRT"; "RRT-star with shortcutting"]; % algorithm names for the result table
found = [RRTStats.found; starStats.found]; % success flags
iterations = [RRTStats.iterations; starStats.iterations]; % iteration counts
nodes = [RRTStats.nodeCount; starStats.nodeCount]; % tree node counts
pathLength = [RRTStats.pathLength; starStats.pathLength]; % final path lengths
runtimeSeconds = [RRTStats.runtime; starStats.runtime]; % runtimes
results = table(algorithm, found, iterations, nodes, pathLength, runtimeSeconds); % MATLAB table of search performance
disp(results); % the performance table

figure("Color", "w", "Name", "Halton RRT Planning Paths"); % trajectory figure
tiledlayout(1, 2, "Padding", "compact", "TileSpacing", "compact"); % side-by-side plots for RRT and RRT-star
nexttile; % first plot tile
plotWorkspace(obstacles, startPosition, goalPosition); % workspace, obstacles, start, and goal
plotTree(RRTTree, [0.65, 0.65, 0.65]); % ordinary RRT exploration tree
plotPath(RRTPath, [0.10, 0.35, 0.95], 2.2); % ordinary RRT solution path
title("RRT trajectory"); % ordinary RRT plot
nexttile; % second plot tile
plotWorkspace(obstacles, startPosition, goalPosition); % workspace, obstacles, start, and goal
plotTree(starTree, [0.72, 0.72, 0.72]); % RRT-star exploration tree
plotPath(starPath, [0.90, 0.25, 0.10], 2.2); % RRT-star solution path
title("RRT-star shortest-path variant"); % bonus plot
saveas(gcf, fullfile(fileparts(mfilename("fullpath")), "halton_rrt_paths.png")); % save the RRT trajectory figure

if ~usejava("desktop") % MATLAB is running in batch mode
    close all; % figures so batch execution exits cleanly
end % batch cleanup

function obstacles = createRandomNonConvexObstacles() % five-obstacle generator
baseCenters = [24, 28; 43, 67; 62, 33; 76, 70; 34, 84]; % nominal obstacle centers
obstacles = cell(1, 5); % cell array for five polygons
for k = 1:5 % the five obstacle slots
    jitter = 4 * (rand(1, 2) - 0.5); % small random shift in both coords
    center = baseCenters(k, :) + jitter; % the random shift to the nominal center
    scaleValue = 8 + 2 * rand; % random obstacle scale
    obstacles{k} = makeConcavePolygon(center, scaleValue); % generated non-convex polygon
end % obstacle generation loop
end % createRandomNonConvexObstacles

function polygon = makeConcavePolygon(center, scaleValue) % counterclockwise L-shaped non-convex polygon
x = center(1); % obstacle center x coord
y = center(2); % obstacle center y coord
r = scaleValue; % obstacle scale with a short local name
polygon = [x - r, y - r; x + r, y - r; x + r, y - 0.15 * r; x + 0.15 * r, y - 0.15 * r; x + 0.15 * r, y + r; x - r, y + r]; % cCW concave polygon
end % makeConcavePolygon

function plotWorkspace(obstacles, startPosition, goalPosition) % common workspace plotting
hold on; % all plotted objects on the same axes
axis([0, 100, 0, 100]); % the workspace limits
axis square; % x and y scales equal
grid on; % show grid lines
for k = 1:numel(obstacles) % all obstacles
    obstacle = obstacles{k}; % one obstacle polygon
    fill(obstacle(:, 1), obstacle(:, 2), [0.18, 0.18, 0.18], "EdgeColor", "k", "FaceAlpha", 0.9); % obstacle
end % obstacle plotting loop
plot(startPosition(1), startPosition(2), "go", "MarkerFaceColor", "g", "MarkerSize", 7); % point
plot(goalPosition(1), goalPosition(2), "mo", "MarkerFaceColor", "m", "MarkerSize", 7); % goal point
xlabel("x"); % x axis
ylabel("y"); % y axis
end % plotWorkspace

function plotTree(tree, colorValue) % tree plotting from node-parent arrays
for k = 2:size(tree.nodes, 1) % every non-root tree node
    parentIndex = tree.parents(k); % parent index
    if parentIndex > 0 % that the node has a valid parent
        segment = [tree.nodes(parentIndex, :); tree.nodes(k, :)]; % parent-child line segment
        plot(segment(:, 1), segment(:, 2), "-", "Color", colorValue, "LineWidth", 0.5); % tree edge
    end % parent validity check
end % tree plotting loop
end % plotTree

function plotPath(path, colorValue, lineWidth) % solution path plotting
if isempty(path) % the path is empty
    text(50, 50, "No path found", "HorizontalAlignment", "center", "FontWeight", "bold"); % report failure directly on the plot
else % path if it exists
    plot(path(:, 1), path(:, 2), "-", "Color", colorValue, "LineWidth", lineWidth); % solution polyline
end % path plotting branch
end % plotPath

