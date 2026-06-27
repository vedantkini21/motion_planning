% Halton RRT Planning
function [path, tree, stats] = RRTStarSearch(startPosition, goalPosition, obstacles) % bonus shortest-path RRT-star function
workspaceBounds = [0, 100, 0, 100]; % workspace bounds as [xmin,xmax,ymin,ymax]
stepSize = 3; % maximum extension distance
goalRadius = 4; % distance that allows connection to the goal
goalBias = 0.15; % probability of sampling the goal directly
rewireRadius = 12; % radius used for better parents and rewiring
maxIterations = 5500; % maximum number of RRT-star iterations
nodes = zeros(maxIterations + 200, 2); % preallocate tree nodes
parents = zeros(maxIterations + 200, 1); % preallocate parent indices
costs = inf(maxIterations + 200, 1); % preallocate path costs from the start
nodes(1, :) = startPosition; % start node
parents(1) = 0; % give the start node no parent
costs(1) = 0; % set the start cost to zero
nodeCount = 1; % track the number of active nodes
goalIndex = 0; % best goal-node index as missing
ticValue = tic; % timing the search
for iteration = 1:maxIterations % RRT-star expansion loop
    sample = sampleWorkspace(goalPosition, goalBias, workspaceBounds); % random sample with goal bias
    nearestIndex = nearestNode(nodes, nodeCount, sample); % find the nearest existing tree node
    newPosition = steerToward(nodes(nearestIndex, :), sample, stepSize); % from the nearest node toward the sample
    if ~edgeCollisionFree(nodes(nearestIndex, :), newPosition, obstacles, workspaceBounds) % initial edge collides
        continue; % reject edge
    end % initial extension validity check
    nearIndices = nearNodes(nodes, nodeCount, newPosition, rewireRadius); % find nearby tree nodes for parent selection
    bestParent = nearestIndex; % best parent as the nearest node
    bestCost = costs(nearestIndex) + norm(newPosition - nodes(nearestIndex, :), 2); % best cost through the nearest node
    for k = 1:numel(nearIndices) % nearby parent candidates
        candidateIndex = nearIndices(k); % one candidate node index
        candidateCost = costs(candidateIndex) + norm(newPosition - nodes(candidateIndex, :), 2); % cost through the candidate
        if candidateCost < bestCost && edgeCollisionFree(nodes(candidateIndex, :), newPosition, obstacles, workspaceBounds) % the candidate improves the path
            bestParent = candidateIndex; % better parent
            bestCost = candidateCost; % better cost
        end % better-parent check
    end % parent-selection loop
    nodeCount = nodeCount + 1; % reserve a new tree node
    nodes(nodeCount, :) = newPosition; % new node coords
    parents(nodeCount) = bestParent; % best parent
    costs(nodeCount) = bestCost; % best cost-to-come
    for k = 1:numel(nearIndices) % nearby nodes for rewiring
        candidateIndex = nearIndices(k); % one nearby node index
        rewiredCost = costs(nodeCount) + norm(nodes(candidateIndex, :) - newPosition, 2); % cost through new node
        if candidateIndex ~= bestParent && rewiredCost < costs(candidateIndex) && edgeCollisionFree(newPosition, nodes(candidateIndex, :), obstacles, workspaceBounds) % rewiring improves the nearby node
            parents(candidateIndex) = nodeCount; % the nearby node to new node
            costs(candidateIndex) = rewiredCost; % improved cost
            [parents, costs] = updateDescendantCosts(nodes, parents, costs, candidateIndex, nodeCount); % cost changes to descendants
        end % rewiring improvement check
    end % rewiring loop
    if norm(newPosition - goalPosition, 2) <= goalRadius % new node is near the goal
        if edgeCollisionFree(newPosition, goalPosition, obstacles, workspaceBounds) % the goal connection is collision-free
            candidateGoalCost = costs(nodeCount) + norm(goalPosition - newPosition, 2); % cost through new node to the goal
            if goalIndex == 0 % this is the first goal connection
                nodeCount = nodeCount + 1; % reserve a node for exact goal
                nodes(nodeCount, :) = goalPosition; % exact goal
                parents(nodeCount) = nodeCount - 1; % new node as the goal parent
                costs(nodeCount) = candidateGoalCost; % new goal cost
                goalIndex = nodeCount; % remember exact goal node
            elseif candidateGoalCost < costs(goalIndex) % this route improves the goal
                parents(goalIndex) = nodeCount; % the existing goal node
                costs(goalIndex) = candidateGoalCost; % improved goal cost
            end % goal-node update
        end % goal connection collision check
    end % goal-neighborhood check
end % RRT-star expansion loop
tree.nodes = nodes(1:nodeCount, :); % store active tree nodes
tree.parents = parents(1:nodeCount); % store active parent indices
tree.costs = costs(1:nodeCount); % store active costs
stats.iterations = iteration; % number of attempted iterations
stats.nodeCount = nodeCount; % number of accepted nodes
stats.runtime = toc(ticValue); % search runtime
stats.found = goalIndex > 0; % whether a goal connection was found
if stats.found % RRT-star found a route
    rawPath = reconstructTreePath(tree.nodes, tree.parents, goalIndex); % the best tree path
    path = shortcutPath(rawPath, obstacles, workspaceBounds); % deterministic shortcut smoothing to further reduce path length
    stats.pathLength = pathLength(path); % final smoothed path length
else % RRT-star failure
    path = []; % empty path on failure
    stats.pathLength = inf; % infinite path length on failure
end % success/failure branch
end % RRTStarSearch

function sample = sampleWorkspace(goalPosition, goalBias, bounds) % random sampling with goal bias
if rand < goalBias % the biased sample should be the goal
    sample = goalPosition; % exact goal as a sample
else % unbiased random workspace sample
    sample = [bounds(1) + rand * (bounds(2) - bounds(1)), bounds(3) + rand * (bounds(4) - bounds(3))]; % random point inside the square
end % sampling branch
end % sampleWorkspace

function index = nearestNode(nodes, nodeCount, sample) % nearest-neighbor search without graph-search functions
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

function indices = nearNodes(nodes, nodeCount, point, radius) % radius-neighbor search for rewiring
indices = zeros(nodeCount, 1); % worst-case output size
count = 0; % number of nearby nodes
for k = 1:nodeCount % all active nodes
    if norm(nodes(k, :) - point, 2) <= radius % the node lies inside the rewiring radius
        count = count + 1; % increase the nearby-node count
        indices(count) = k; % nearby-node index
    end % radius check
end % active-node loop
indices = indices(1:count); % the output to the actual nearby nodes
end % nearNodes

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

function [parents, costs] = updateDescendantCosts(nodes, parents, costs, rootIndex, nodeCount) % descendant cost propagation after rewiring
for k = 1:nodeCount % active nodes only
    if parents(k) == rootIndex % this node is a direct child of the rewired root
        costs(k) = costs(rootIndex) + norm(nodes(k, :) - nodes(rootIndex, :), 2); % the child cost from the rewired root
        [parents, costs] = updateDescendantCosts(nodes, parents, costs, k, nodeCount); % recursively update descendants of the child
    end % direct-child check
end % active-node loop
end % updateDescendantCosts

function path = shortcutPath(path, obstacles, bounds) % deterministic path shortcutting for shortest-path improvement
if size(path, 1) <= 2 % shortcutting is unnecessary
    return; % leave short paths unchanged
end % short-path check
changed = true; % shortcut improvement flag
while changed % repeat while a shortcut was found
    changed = false; % reset the shortcut flag
    i = 1; % at the first waypoint
    while i <= size(path, 1) - 2 % while at least one intermediate waypoint exists
        j = size(path, 1); % connecting to the farthest later waypoint first
        while j >= i + 2 % until no intermediate point would be removed
            if edgeCollisionFree(path(i, :), path(j, :), obstacles, bounds) % a direct shortcut is collision-free
                path = [path(1:i, :); path(j:end, :)]; % the bypassed waypoints
                changed = true; % that a shortcut was accepted
                break; % restart from current waypoint
            end % shortcut validity check
            j = j - 1; % the next closer waypoint
        end % destination waypoint loop
        i = i + 1; % advance to the next source waypoint
    end % source waypoint loop
end % shortcut improvement loop
end % shortcutPath

