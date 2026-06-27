% Halton RRT Planning
function [path, tree, stats] = RRTSearch(startPosition, goalPosition, obstacles) % required RRT search function
workspaceBounds = [0, 100, 0, 100]; % workspace bounds as [xmin,xmax,ymin,ymax]
stepSize = 3; % maximum extension distance
goalRadius = 4; % distance that allows connection to the goal
goalBias = 0.12; % probability of sampling the goal directly
maxIterations = 4500; % maximum number of RRT iterations
nodes = zeros(maxIterations + 2, 2); % preallocate tree nodes
parents = zeros(maxIterations + 2, 1); % preallocate parent indices
costs = inf(maxIterations + 2, 1); % preallocate path costs from the start
nodes(1, :) = startPosition; % start node
parents(1) = 0; % give the start node no parent
costs(1) = 0; % set the start cost to zero
nodeCount = 1; % track the number of active nodes
goalIndex = 0; % goal-node index as missing
ticValue = tic; % timing the search
for iteration = 1:maxIterations % RRT expansion loop
    sample = sampleWorkspace(goalPosition, goalBias, workspaceBounds); % random sample with goal bias
    nearestIndex = nearestNode(nodes, nodeCount, sample); % find the nearest existing tree node
    newPosition = steerToward(nodes(nearestIndex, :), sample, stepSize); % from the nearest node toward the sample
    if ~edgeCollisionFree(nodes(nearestIndex, :), newPosition, obstacles, workspaceBounds) % extension collides
        continue; % reject edge
    end % extension validity check
    nodeCount = nodeCount + 1; % reserve a new tree node
    nodes(nodeCount, :) = newPosition; % new node coords
    parents(nodeCount) = nearestIndex; % nearest node as parent
    costs(nodeCount) = costs(nearestIndex) + norm(newPosition - nodes(nearestIndex, :), 2); % new cost-to-come
    if norm(newPosition - goalPosition, 2) <= goalRadius % the tree has reached the goal region
        if edgeCollisionFree(newPosition, goalPosition, obstacles, workspaceBounds) % the goal connection is collision-free
            nodeCount = nodeCount + 1; % reserve a node for exact goal
            nodes(nodeCount, :) = goalPosition; % exact goal node
            parents(nodeCount) = nodeCount - 1; % new position as the goal parent
            costs(nodeCount) = costs(nodeCount - 1) + norm(goalPosition - newPosition, 2); % exact goal cost
            goalIndex = nodeCount; % remember exact goal node index
            break; % first feasible path
        end % goal connection collision check
    end % goal-region check
end % RRT expansion loop
tree.nodes = nodes(1:nodeCount, :); % store the active tree nodes
tree.parents = parents(1:nodeCount); % store the active parent array
tree.costs = costs(1:nodeCount); % store active costs
stats.iterations = iteration; % number of attempted iterations
stats.nodeCount = nodeCount; % number of accepted nodes
stats.runtime = toc(ticValue); % search runtime
stats.found = goalIndex > 0; % whether the goal was reached
if stats.found % a path exists
    path = reconstructTreePath(tree.nodes, tree.parents, goalIndex); % the start-to-goal path
    stats.pathLength = pathLength(path); % path length
else % RRT failure
    path = []; % empty path on failure
    stats.pathLength = inf; % infinite path length on failure
end % success/failure branch
end % RRTSearch

function sample = sampleWorkspace(goalPosition, goalBias, bounds) % random sampling with goal bias
if rand < goalBias % the biased sample should be the goal
    sample = goalPosition; % exact goal as a sample
else % unbiased random workspace sample
    sample = [bounds(1) + rand * (bounds(2) - bounds(1)), bounds(3) + rand * (bounds(4) - bounds(3))]; % random point inside the square
end % sampling branch
end % sampleWorkspace

function index = nearestNode(nodes, nodeCount, sample) % nearest-neighbor search without built-in graph tools
bestDistance = inf; % best distance
index = 1; % best node index
for k = 1:nodeCount % all active nodes
    currentDistance = sum((nodes(k, :) - sample).^2); % squared Euclidean distance to the sample
    if currentDistance < bestDistance % current node is closer
        bestDistance = currentDistance; % improved distance
        index = k; % improved node index
    end % nearest-node update
end % active-node loop
end % nearestNode

function newPosition = steerToward(fromPosition, toPosition, stepSize) % bounded RRT steering
direction = toPosition - fromPosition; % vector from the tree node to the sample
distanceValue = norm(direction, 2); % euclidean distance to the sample
if distanceValue <= stepSize % the sample is within one step
    newPosition = toPosition; % directly to the sample
elseif distanceValue < 1e-12 % the sample equals the source point
    newPosition = fromPosition; % stay at the source point
else % one step toward the sample
    newPosition = fromPosition + stepSize * direction / distanceValue; % bounded extension point
end % steering branch
end % steerToward

function free = edgeCollisionFree(fromPosition, toPosition, obstacles, bounds) % swept-edge collision checking
distanceValue = norm(toPosition - fromPosition, 2); % edge length
sampleCount = max(2, ceil(distanceValue / 0.75) + 1); % enough interpolation samples for the triangle robot
free = true; % edge as collision-free
for k = 1:sampleCount % every interpolation sample
    alpha = (k - 1) / (sampleCount - 1); % interpolation fraction
    position = (1 - alpha) * fromPosition + alpha * toPosition; % interpolated robot centroid
    if robotInCollision(position, obstacles, bounds) % robot-obstacle or robot-boundary collision
        free = false; % edge as colliding
        return; % after the first collision
    end % robot-collision check
end % interpolation loop
end % edgeCollisionFree

function collision = robotInCollision(position, obstacles, bounds) % triangular robot collision against all obstacles
robot = triangleRobot(position); % translated triangular robot polygon
collision = false; % collision as false
if any(robot(:, 1) < bounds(1)) || any(robot(:, 1) > bounds(2)) || any(robot(:, 2) < bounds(3)) || any(robot(:, 2) > bounds(4)) % workspace boundary violation
    collision = true; % boundary violation as collision
    return; % after boundary collision
end % boundary check
for k = 1:numel(obstacles) % all obstacle polygons
    if collisionDetection(robot, obstacles{k}) % robot polygon against the obstacle polygon
        collision = true; % obstacle contact as collision
        return; % after obstacle collision
    end % obstacle collision check
end % obstacle loop
end % robotInCollision

function robot = triangleRobot(position) % fixed-orientation equilateral triangle robot
sideLength = 3; % robot side length
heightValue = sideLength * sqrt(3) / 2; % equilateral-triangle height
localVertices = [0, 2 * heightValue / 3; -sideLength / 2, -heightValue / 3; sideLength / 2, -heightValue / 3]; % centroid-centered vertices
robot = localVertices + position; % translate the robot vertices to the centroid
end % triangleRobot

function path = reconstructTreePath(nodes, parents, goalIndex) % tree path reconstruction
path = nodes(goalIndex, :); % the path at the goal node
currentIndex = goalIndex; % backtracking from the goal index
while parents(currentIndex) ~= 0 % until the start node is reached
    currentIndex = parents(currentIndex); % to the parent node
    path = [nodes(currentIndex, :); path]; % #ok<AGROW> % grow path
end % parent backtracking loop
end % reconstructTreePath

function lengthValue = pathLength(path) % polyline path-length computation
if isempty(path) % the path is empty
    lengthValue = inf; % infinite length for failure
else % length for a valid path
    differences = diff(path, 1, 1); % consecutive segment vectors
    lengthValue = sum(sqrt(sum(differences.^2, 2))); % sum Euclidean segment lengths
end % empty path branch
end % pathLength

