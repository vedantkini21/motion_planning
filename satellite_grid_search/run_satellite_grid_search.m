% Satellite Grid Search
clearvars; % fresh workspace
close all; % close figures
clc; % clear console

rng(7); % reproducible map
gridWidth = 60; % grid width
gridHeight = 50; % grid height
obstacleCount = 200; % obstacle count
initialPosition = [5, 5]; % reported start
goalPosition = [56, 44]; % goal position
delayedPosition = [18, 17]; % delayed start
protectedCells = [initialPosition; goalPosition; delayedPosition]; % protected cells

map = createCourseMap(gridWidth, gridHeight, obstacleCount, protectedCells); % binary occupancy grid
satelliteResult = satelliteShortestPath(map, initialPosition, goalPosition); % satellite planner
astarManhattan = astarVehicle(map, delayedPosition, goalPosition, "manhattan", satelliteResult.path); % Manhattan A-star
astarEuclidean = astarVehicle(map, delayedPosition, goalPosition, "euclidean", satelliteResult.path); % Euclidean A-star
astarSatellite = astarVehicle(map, delayedPosition, goalPosition, "satellite", satelliteResult.path); % satellite-path A-star

fprintf("\nState space: S = {(x,y) | 1 <= x <= %d, 1 <= y <= %d, occupancy(x,y)=0}.\n", gridWidth, gridHeight); % state-space definition
fprintf("Action space: A = {[1,0],[-1,0],[0,1],[0,-1]}.\n"); % action-space definition
fprintf("Transition: s_{k+1} = s_k + a_k if the target cell is inside the grid and free.\n\n"); % transition equation

efficiencyTable = buildEfficiencyTable(satelliteResult, astarManhattan, astarEuclidean, astarSatellite); % efficiency table
disp(efficiencyTable); % print table

plotResults(map, initialPosition, delayedPosition, goalPosition, satelliteResult, astarManhattan, astarEuclidean, astarSatellite); % plot paths

function map = createCourseMap(width, height, obstacleCount, protectedCells) % binary occupancy grid generator
    if exist("binaryOccupancyMap", "class") == 8 || exist("binaryOccupancyMap", "file") == 2 % Robotics System Toolbox is available
        map = binaryOccupancyMap(width, height, 1, "grid"); % MATLAB occupancy map when available
    else % numeric fallback
        map = zeros(height, width); % plain binary matrix with rows as y and columns as x
    end % map type selection
    placed = 0; % obstacle counter
    while placed < obstacleCount % fill obstacles
        cellCandidate = [randi(width), randi(height)]; % random grid cell as [x,y]
        if isProtectedCell(cellCandidate, protectedCells) % protected cell
            continue; % protected cells
        end % protected-cell check
        if isOccupiedCell(map, cellCandidate) % duplicate cell
            continue; % repeated obstacle samples
        end % duplicate-obstacle check
        map = setCellOccupied(map, cellCandidate, 1); % sampled cell as occupied
        placed = placed + 1; % update count
    end % obstacle placement loop
    for k = 1:size(protectedCells, 1) % clear protected cells
        map = setCellOccupied(map, protectedCells(k, :), 0); % mark free
    end % protected-cell clearing loop
end % createCourseMap

function tf = isProtectedCell(cellCandidate, protectedCells) % helper for protected cell lookup
    tf = any(all(protectedCells == cellCandidate, 2)); % true when the candidate matches a protected row
end % isProtectedCell

function result = satelliteShortestPath(map, startCell, goalCell) % satellite planning routine
    [width, height] = getMapSize(map); % map dimensions
    Sat1Region = @(cell) cell(1) >= 1 && cell(1) <= 40 && cell(2) >= 1 && cell(2) <= height; % Sat1's visible x range
    Sat2Region = @(cell) cell(1) >= 30 && cell(1) <= width && cell(2) >= 1 && cell(2) <= height; % Sat2's visible x range
    overlapCells = buildOverlapCells(height); % cells that may be shared between satellites
    candidates = {}; % cell array for candidate complete paths
    candidateNames = strings(0, 1); % string array for candidate labels
    Sat1Search = emptySearchStats("Sat1 Dijkstra"); % default Sat1 search record
    Sat2Search = emptySearchStats("Sat2 BFS"); % default Sat2 search record
    if Sat1Region(startCell) % Sat1 can see the satellite-reported start
        Sat1FromStart = searchGrid8(map, startCell, Sat1Region, "Dijkstra", "Sat1 Dijkstra"); % Dijkstra over Sat1's map
        Sat1Search = Sat1FromStart; % Sat1 search used for reporting
        if Sat1Region(goalCell) && isfinite(getCostAt(Sat1FromStart, goalCell)) % Sat1 alone reaches the goal
            candidates{end + 1} = reconstructPath(Sat1FromStart, goalCell); % direct Sat1 path
            candidateNames(end + 1, 1) = "Sat1 direct"; % direct Sat1 path
        end % direct Sat1 candidate check
        if Sat2Region(goalCell) % Sat2 can see final goal
            Sat2FromGoal = searchGrid8(map, goalCell, Sat2Region, "BFS", "Sat2 BFS"); % BFS backward from the goal over Sat2's map
            Sat2Search = Sat2FromGoal; % Sat2 search used for reporting
            [bridgePath, bridgeName] = bestBridgePath(Sat1FromStart, Sat2FromGoal, overlapCells, "Sat1-to-Sat2"); % paths through the overlap
            if ~isempty(bridgePath) % a valid bridge was found
                candidates{end + 1} = bridgePath; % bridged satellite path
                candidateNames(end + 1, 1) = bridgeName; % bridged satellite path
            end % bridge validity check
        end % Sat2 goal visibility check
    end % Sat1-start branch
    if Sat2Region(startCell) % Sat2 can see the satellite-reported start
        Sat2FromStart = searchGrid8(map, startCell, Sat2Region, "BFS", "Sat2 BFS"); % BFS over Sat2's map
        Sat2Search = Sat2FromStart; % Sat2 search used for reporting
        if Sat2Region(goalCell) && isfinite(getCostAt(Sat2FromStart, goalCell)) % Sat2 alone reaches the goal
            candidates{end + 1} = reconstructPath(Sat2FromStart, goalCell); % direct Sat2 path
            candidateNames(end + 1, 1) = "Sat2 direct"; % direct Sat2 path
        end % direct Sat2 candidate check
        if Sat1Region(goalCell) % Sat1 can see final goal
            Sat1FromGoal = searchGrid8(map, goalCell, Sat1Region, "Dijkstra", "Sat1 Dijkstra"); % Dijkstra backward from the goal over Sat1's map
            Sat1Search = Sat1FromGoal; % Sat1 search used for reporting
            [bridgePath, bridgeName] = bestBridgePath(Sat2FromStart, Sat1FromGoal, overlapCells, "Sat2-to-Sat1"); % paths through the overlap
            if ~isempty(bridgePath) % a valid bridge was found
                candidates{end + 1} = bridgePath; % bridged satellite path
                candidateNames(end + 1, 1) = bridgeName; % bridged satellite path
            end % bridge validity check
        end % Sat1 goal visibility check
    end % Sat2-start branch
    [path, pathName, pathCost] = chooseBestPath(candidates, candidateNames); % lowest-cost complete candidate
    result.path = path; % chosen satellite-system path
    result.name = pathName; % chosen path label
    result.cost = pathCost; % chosen path cost
    result.found = ~isempty(path); % whether the satellite system found a path
    result.Sat1Stats = Sat1Search; % Sat1 efficiency statistics
    result.Sat2Stats = Sat2Search; % Sat2 efficiency statistics
    if ~result.found % no satellite path exists
        warning("Satellite planner failed to find a path."); % report satellite planning failure
    end % failure warning
end % satelliteShortestPath

function cells = buildOverlapCells(height) % overlap-cell builder
    cells = zeros(11 * height, 2); % all cells with x in 30 through 40
    row = 1; % filling at the first row
    for x = 30:40 % the shared x coords
        for y = 1:height % every y coord
            cells(row, :) = [x, y]; % one overlap cell
            row = row + 1; % advance the output row
        end % y loop
    end % x loop
end % buildOverlapCells

function [bridgePath, bridgeName] = bestBridgePath(firstSearch, secondSearch, overlapCells, bridgeName) % overlap combiner
    bestCost = inf; % best combined cost
    bridgePath = []; % best bridge path
    for k = 1:size(overlapCells, 1) % each shareable overlap node
        bridgeCell = overlapCells(k, :); % current overlap cell
        firstCost = getCostAt(firstSearch, bridgeCell); % first satellite's cost to the bridge
        secondCost = getCostAt(secondSearch, bridgeCell); % second satellite's reverse cost to the bridge
        if ~isfinite(firstCost) || ~isfinite(secondCost) % either satellite failed to visit the bridge
            continue; % unreachable bridge cells
        end % reachability check
        firstPath = reconstructPath(firstSearch, bridgeCell); % start-to-bridge path from the first search
        secondReversePath = reconstructPath(secondSearch, bridgeCell); % goal-to-bridge path from the second search
        secondPath = flipud(secondReversePath); % the second path to obtain bridge-to-goal order
        candidatePath = [firstPath; secondPath(2:end, :)]; % the two paths without repeating the bridge
        candidateCost = pathCost8(candidatePath); % the geometric eight-connected path cost
        if candidateCost < bestCost % this bridge is better
            bestCost = candidateCost; % new best cost
            bridgePath = candidatePath; % new best path
        end % best-candidate update
    end % overlap loop
end % bestBridgePath

function [bestPath, bestName, bestCost] = chooseBestPath(candidates, candidateNames) % candidate selector
    bestPath = []; % selected path as empty
    bestName = "No path"; % selected label
    bestCost = inf; % selected cost
    for k = 1:numel(candidates) % every candidate path
        candidatePath = candidates{k}; % candidate path
        candidateCost = pathCost8(candidatePath); % eight-connected geometric path cost
        if candidateCost < bestCost % this candidate is best so far
            bestPath = candidatePath; % best path
            bestName = candidateNames(k); % best label
            bestCost = candidateCost; % best cost
        end % best-candidate update
    end % candidate loop
end % chooseBestPath

function search = searchGrid8(map, startCell, regionFunction, method, label) % BFS and Dijkstra over an eight-connected grid
    timerValue = tic; % timing the graph search
    [width, height] = getMapSize(map); % map dimensions
    cost = inf(height, width); % cost-to-come values
    parentX = nan(height, width); % parent x coords
    parentY = nan(height, width); % parent y coords
    closed = false(height, width); % expanded-node mask
    opened = false(height, width); % discovered-node mask
    queue = zeros(width * height, 2); % fixed-size queue for BFS
    head = 1; % BFS queue head
    tail = 1; % BFS queue tail
    cost(startCell(2), startCell(1)) = 0; % set the start cost to zero
    opened(startCell(2), startCell(1)) = true; % as discovered
    queue(tail, :) = startCell; % start into the BFS queue
    expanded = 0; % expanded-node counter
    while true % repeat until no open node remains
        if method == "BFS" % current method is BFS
            if head > tail % the BFS queue is empty
                break; % the search when no node remains
            end % empty queue check
            current = queue(head, :); % the oldest queued cell
            head = head + 1; % advance the BFS queue head
            if closed(current(2), current(1)) % this queued cell is stale
                continue; % stale cells
            end % stale-cell check
        else % the Dijkstra branch for weighted search
            openCosts = cost; % all cost values for masked minimization
            openCosts(closed) = inf; % already expanded nodes from consideration
            openCosts(~opened) = inf; % undiscovered nodes from consideration
            [minimumCost, linearIndex] = min(openCosts(:)); % open node with smallest cost
            if ~isfinite(minimumCost) % the Dijkstra open set is empty
                break; % the search when no node remains
            end % empty open-set check
            [currentY, currentX] = ind2sub([height, width], linearIndex); % linear index to grid coords
            current = [currentX, currentY]; % selected current cell as [x,y]
        end % method-specific pop
        closed(current(2), current(1)) = true; % current cell as expanded
        expanded = expanded + 1; % expanded-node counter
        neighbors = neighbors8(current); % eight-connected neighbors
        for n = 1:size(neighbors, 1) % each neighbor
            neighbor = neighbors(n, :); % one neighbor cell
            if ~isValidFreeCell(map, neighbor) % map bounds and obstacle occupancy
                continue; % invalid or occupied neighbors
            end % validity check
            if ~regionFunction(neighbor) % the satellite can see this neighbor
                continue; % cells outside the satellite view
            end % region check
            if closed(neighbor(2), neighbor(1)) % the neighbor has already been expanded
                continue; % expanded neighbors
            end % closed-neighbor check
            if method == "BFS" % this search is BFS
                stepCost = 1; % unit edge cost for BFS
            else % weighted costs for Dijkstra
                stepCost = norm(neighbor - current, 2); % euclidean edge cost for Dijkstra
            end % edge-cost selection
            tentativeCost = cost(current(2), current(1)) + stepCost; % tentative cost-to-come
            if tentativeCost < cost(neighbor(2), neighbor(1)) % new route improves the neighbor
                cost(neighbor(2), neighbor(1)) = tentativeCost; % improved cost
                parentX(neighbor(2), neighbor(1)) = current(1); % parent x coord
                parentY(neighbor(2), neighbor(1)) = current(2); % parent y coord
                if ~opened(neighbor(2), neighbor(1)) % this is the first discovery
                    opened(neighbor(2), neighbor(1)) = true; % neighbor as discovered
                    tail = tail + 1; % advance the queue tail
                    queue(tail, :) = neighbor; % the neighbor for BFS
                end % discovery check
            end % relaxation check
        end % neighbor loop
    end % search loop
    search.cost = cost; % final cost grid
    search.parentX = parentX; % parent x grid
    search.parentY = parentY; % parent y grid
    search.expanded = expanded; % number of expanded nodes
    search.visited = nnz(opened); % number of discovered nodes
    search.runtime = toc(timerValue); % search runtime
    search.label = label; % search label
end % searchGrid8

function astar = astarVehicle(map, startCell, goalCell, heuristicName, satellitePath) % vehicle A-star with local sensing
    timerValue = tic; % timing A-star
    [width, height] = getMapSize(map); % map dimensions
    gScore = inf(height, width); % cost-to-come values
    fScore = inf(height, width); % estimated total costs
    parentX = nan(height, width); % parent x coords
    parentY = nan(height, width); % parent y coords
    openSet = false(height, width); % open set
    closedSet = false(height, width); % closed set
    localKnowledge = nan(height, width); % unknown cells as NaN, free cells as 0, and obstacles as 1
    remainingSatelliteCost = satelliteRemainingCost4(satellitePath); % path-to-goal costs for the satellite heuristic
    gScore(startCell(2), startCell(1)) = 0; % set the start cost to zero
    fScore(startCell(2), startCell(1)) = heuristicValue(startCell, goalCell, heuristicName, satellitePath, remainingSatelliteCost); % set the start f score
    openSet(startCell(2), startCell(1)) = true; % start into the open set
    expanded = 0; % expanded-node counter
    while any(openSet(:)) % while the open set is nonempty
        maskedFScore = fScore; % f scores for masked minimization
        maskedFScore(~openSet) = inf; % cells outside the open set
        [~, linearIndex] = min(maskedFScore(:)); % open cell with minimum f score
        [currentY, currentX] = ind2sub([height, width], linearIndex); % linear index to grid coords
        current = [currentX, currentY]; % current cell as [x,y]
        if isequal(current, goalCell) % the goal has been reached
            break; % A-star after selecting the goal
        end % goal check
        openSet(current(2), current(1)) = false; % current cell from the open set
        closedSet(current(2), current(1)) = true; % current cell to the closed set
        expanded = expanded + 1; % expanded-node counter
        localKnowledge = senseLocalCells(map, localKnowledge, current); % current and neighboring cells
        neighbors = neighbors4(current); % vehicle's four-connected neighbors
        for n = 1:size(neighbors, 1) % each possible vehicle action
            neighbor = neighbors(n, :); % one neighboring state
            if ~isInsideMap(map, neighbor) % the neighbor is inside the map
                continue; % outside states
            end % bounds check
            if isnan(localKnowledge(neighbor(2), neighbor(1))) % the neighbor is not yet sensed
                localKnowledge = senseLocalCells(map, localKnowledge, current); % ensure current-neighborhood sensing has been applied
            end % unknown-neighbor check
            if localKnowledge(neighbor(2), neighbor(1)) > 0.5 % the sensed neighbor is occupied
                continue; % occupied cells
            end % obstacle check
            if closedSet(neighbor(2), neighbor(1)) % the neighbor has already been expanded
                continue; % closed cells
            end % closed-neighbor check
            tentativeGScore = gScore(current(2), current(1)) + 1; % one unit for a forward/backward/left/right move
            if tentativeGScore < gScore(neighbor(2), neighbor(1)) % new path improves the neighbor
                parentX(neighbor(2), neighbor(1)) = current(1); % parent x coord
                parentY(neighbor(2), neighbor(1)) = current(2); % parent y coord
                gScore(neighbor(2), neighbor(1)) = tentativeGScore; % improved cost-to-come
                fScore(neighbor(2), neighbor(1)) = tentativeGScore + heuristicValue(neighbor, goalCell, heuristicName, satellitePath, remainingSatelliteCost); % improved f score
                openSet(neighbor(2), neighbor(1)) = true; % or keep the neighbor in the open set
            end % relaxation check
        end % neighbor loop
    end % A-star loop
    astarSearch.cost = gScore; % A-star g-score grid for reconstruction
    astarSearch.parentX = parentX; % parent x grid for reconstruction
    astarSearch.parentY = parentY; % parent y grid for reconstruction
    astar.path = reconstructPath(astarSearch, goalCell); % reconstruct the goal path if it exists
    astar.found = ~isempty(astar.path); % whether A-star found a route
    astar.cost = getCostAt(astarSearch, goalCell); % path cost to the goal
    astar.expanded = expanded; % expanded-node count
    astar.visited = nnz(isfinite(gScore)); % discovered-node count
    astar.runtime = toc(timerValue); % runtime
    astar.heuristic = heuristicName; % heuristic name
    if ~astar.found % A-star failed
        warning("A-star with %s heuristic failed to find a path.", heuristicName); % report A-star failure
    end % failure warning
end % astarVehicle

function localKnowledge = senseLocalCells(map, localKnowledge, current) % local obstacle sensing
    cellsToSense = [current; neighbors4(current)]; % current cell and all four neighboring cells
    for k = 1:size(cellsToSense, 1) % each sensed cell
        cellToSense = cellsToSense(k, :); % sensed cell
        if isInsideMap(map, cellToSense) % the sensed cell is inside the map
            localKnowledge(cellToSense(2), cellToSense(1)) = double(isOccupiedCell(map, cellToSense)); % sensed occupancy value
        end % bounds check
    end % sensing loop
end % senseLocalCells

function h = heuristicValue(cell, goalCell, heuristicName, satellitePath, remainingSatelliteCost) % all A-star heuristic functions
    delta = abs(goalCell - cell); % absolute coord differences to the goal
    if heuristicName == "manhattan" % manhattan distance is requested
        h = delta(1) + delta(2); % four-connected shortest distance without obstacles
    elseif heuristicName == "euclidean" % euclidean distance is requested
        h = norm(goalCell - cell, 2); % straight-line distance to the goal
    elseif heuristicName == "satellite" && ~isempty(satellitePath) % the satellite-path heuristic is available
        distancesToPath = sqrt(sum((satellitePath - cell).^2, 2)); % euclidean distance from the cell to every satellite path point
        h = min(distancesToPath + remainingSatelliteCost); % the best path point plus its remaining path cost
    else % when a requested heuristic cannot be evaluated
        h = norm(goalCell - cell, 2); % euclidean distance as a safe fallback
    end % heuristic selection
end % heuristicValue

function remainingCost = satelliteRemainingCost4(satellitePath) % cost-to-go along the reported satellite path
    if isempty(satellitePath) % no satellite path is available
        remainingCost = []; % empty vector
        return; % leave the helper early
    end % empty path check
    remainingCost = zeros(size(satellitePath, 1), 1); % remaining-cost vector
    for k = size(satellitePath, 1)-1:-1:1 % traverse the satellite path backward
        step = abs(satellitePath(k + 1, :) - satellitePath(k, :)); % grid displacement to the next path point
        remainingCost(k) = remainingCost(k + 1) + step(1) + step(2); % count diagonal satellite moves as two vehicle moves
    end % backward cost loop
end % satelliteRemainingCost4

function tableOut = buildEfficiencyTable(satelliteResult, astarManhattan, astarEuclidean, astarSatellite) % efficiency table builder
    algorithm = ["Sat1 Dijkstra"; "Sat2 BFS"; "Satellite combined"; "A-star Manhattan"; "A-star Euclidean"; "A-star Satellite"]; % algorithm labels
    expanded = [satelliteResult.Sat1Stats.expanded; satelliteResult.Sat2Stats.expanded; NaN; astarManhattan.expanded; astarEuclidean.expanded; astarSatellite.expanded]; % expanded counts
    visited = [satelliteResult.Sat1Stats.visited; satelliteResult.Sat2Stats.visited; NaN; astarManhattan.visited; astarEuclidean.visited; astarSatellite.visited]; % visited counts
    pathCost = [NaN; NaN; satelliteResult.cost; astarManhattan.cost; astarEuclidean.cost; astarSatellite.cost]; % path costs where meaningful
    runtimeSeconds = [satelliteResult.Sat1Stats.runtime; satelliteResult.Sat2Stats.runtime; NaN; astarManhattan.runtime; astarEuclidean.runtime; astarSatellite.runtime]; % runtimes
    found = [satelliteResult.Sat1Stats.expanded > 0; satelliteResult.Sat2Stats.expanded > 0; satelliteResult.found; astarManhattan.found; astarEuclidean.found; astarSatellite.found]; % success flags
    tableOut = table(algorithm, expanded, visited, pathCost, runtimeSeconds, found); % MATLAB table
end % buildEfficiencyTable

function plotResults(map, initialPosition, delayedPosition, goalPosition, satelliteResult, astarManhattan, astarEuclidean, astarSatellite) % plotting routine
    [width, height] = getMapSize(map); % map dimensions
    occupancyMatrix = mapToMatrix(map); % occupancy map to a plottable matrix
    figure("Color", "w", "Name", "Satellite Grid Search"); % clean figure window
    tiledlayout(1, 2, "Padding", "compact", "TileSpacing", "compact"); % side-by-side axes
    nexttile; % satellite-path tile
    imagesc([1, width], [1, height], occupancyMatrix); % occupied and free cells
    set(gca, "YDir", "normal"); % make y increase upward
    axis equal tight; % square grid cells
    colormap(gca, gray); % grayscale for the binary occupancy map
    hold on; % the map while drawing paths
    plotPath(satelliteResult.path, [0.90, 0.10, 0.10], 2.0); % satellite-system path
    plot(initialPosition(1), initialPosition(2), "go", "MarkerFaceColor", "g", "MarkerSize", 7); % satellite-reported start
    plot(goalPosition(1), goalPosition(2), "mo", "MarkerFaceColor", "m", "MarkerSize", 7); % goal
    xline(40.5, "b--", "Sat1 limit"); % Sat1's upper x limit
    xline(29.5, "c--", "Sat2 limit"); % Sat2's lower x limit
    title("Satellite path"); % satellite tile
    xlabel("x"); % x axis
    ylabel("y"); % y axis
    legend("Satellite path", "Reported start", "Goal", "Location", "southoutside"); % compact legend
    nexttile; % vehicle-replanning tile
    imagesc([1, width], [1, height], occupancyMatrix); % occupied and free cells again
    set(gca, "YDir", "normal"); % make y increase upward
    axis equal tight; % square grid cells
    colormap(gca, gray); % grayscale for the map
    hold on; % the map while drawing paths
    plotPath(astarManhattan.path, [0.10, 0.45, 0.95], 1.6); % manhattan A-star path
    plotPath(astarEuclidean.path, [0.10, 0.65, 0.20], 1.6); % euclidean A-star path
    plotPath(astarSatellite.path, [0.95, 0.55, 0.10], 1.6); % satellite-heuristic A-star path
    plot(delayedPosition(1), delayedPosition(2), "ko", "MarkerFaceColor", "y", "MarkerSize", 7); % delayed current position
    plot(goalPosition(1), goalPosition(2), "mo", "MarkerFaceColor", "m", "MarkerSize", 7); % goal
    title("Vehicle A-star replanning"); % vehicle tile
    xlabel("x"); % x axis
    ylabel("y"); % y axis
    legend("Manhattan", "Euclidean", "Satellite heuristic", "Delayed start", "Goal", "Location", "southoutside"); % compact legend
    outputFile = fullfile(fileparts(mfilename("fullpath")), "satellite_grid_search_paths.png"); % output image path
    saveas(gcf, outputFile); % save the plotted paths as a PNG file
    if ~usejava("desktop") % MATLAB is running in batch batch mode
        close(gcf); % the figure so batch runs can exit cleanly
    end % batch cleanup
end % plotResults

function plotPath(path, colorValue, lineWidth) % safe path plotting helper
    if isempty(path) % the path is empty
        return; % do not draw failed paths
    end % empty path check
    plot(path(:, 1), path(:, 2), "-", "Color", colorValue, "LineWidth", lineWidth); % path polyline
end % plotPath

function path = reconstructPath(search, goalCell) % path reconstruction from parent arrays
    if ~isfinite(getCostAt(search, goalCell)) % the goal is unreachable
        path = []; % empty path on failure
        return; % leave the helper early
    end % unreachable-goal check
    path = goalCell; % reconstruction at the goal cell
    current = goalCell; % current backtracking cell
    while true % backtrack until the start is reached
        parent = [search.parentX(current(2), current(1)), search.parentY(current(2), current(1))]; % parent cell
        if any(isnan(parent)) % this cell has no parent
            break; % at the search root
        end % root check
        path = [parent; path]; % #ok<AGROW> % grow path
        current = parent; % backtracking from the parent
    end % backtracking loop
end % reconstructPath

function costValue = getCostAt(search, cell) % safe cost-grid indexing
    [height, width] = size(search.cost); % cost-grid dimensions
    if cell(1) < 1 || cell(1) > width || cell(2) < 1 || cell(2) > height % the cell is outside the grid
        costValue = inf; % infinite cost outside the grid
    else % valid grid coords
        costValue = search.cost(cell(2), cell(1)); % stored cost at [x,y]
    end % bounds branch
end % getCostAt

function costValue = pathCost8(path) % geometric cost for eight-connected paths
    if isempty(path) % the path is empty
        costValue = inf; % infinite cost for failed paths
        return; % leave the helper early
    end % empty path check
    if size(path, 1) == 1 % the path has only one cell
        costValue = 0; % zero cost for a one-cell path
        return; % leave the helper early
    end % one-cell check
    differences = diff(path, 1, 1); % consecutive path displacements
    costValue = sum(sqrt(sum(differences.^2, 2))); % sum Euclidean step lengths
end % pathCost8

function neighbors = neighbors8(cell) % eight-connected neighbor generation
    offsets = [-1, -1; 0, -1; 1, -1; -1, 0; 1, 0; -1, 1; 0, 1; 1, 1]; % all eight moves
    neighbors = cell + offsets; % offsets to current cell
end % neighbors8

function neighbors = neighbors4(cell) % four-connected vehicle neighbor generation
    offsets = [1, 0; -1, 0; 0, 1; 0, -1]; % forward, backward, left, and right moves
    neighbors = cell + offsets; % offsets to current cell
end % neighbors4

function tf = isValidFreeCell(map, cell) % combined map validity and occupancy check
    tf = isInsideMap(map, cell) && ~isOccupiedCell(map, cell); % true only for in-bounds free cells
end % isValidFreeCell

function tf = isInsideMap(map, cell) % map boundary checking
    [width, height] = getMapSize(map); % map dimensions
    tf = cell(1) >= 1 && cell(1) <= width && cell(2) >= 1 && cell(2) <= height; % x and y limits
end % isInsideMap

function tf = isOccupiedCell(map, cell) % occupancy lookup for both map representations
    if ~isInsideMap(map, cell) % the cell is outside the grid
        tf = true; % outside cells as occupied
        return; % leave the helper early
    end % outside-cell check
    if isnumeric(map) % the map is a numeric matrix
        tf = map(cell(2), cell(1)) > 0.5; % matrix occupancy using row y and column x
    else % Robotics System Toolbox occupancy-map access
        try % grid-coord lookup first
            tf = getOccupancy(map, cell, "grid") > 0.5; % occupancy map occupancy
        catch % for older MATLAB signatures
            tf = getOccupancy(map, cell) > 0.5; % occupancy without the grid argument
        end % occupancy lookup try-catch
    end % representation branch
end % isOccupiedCell

function map = setCellOccupied(map, cell, value) % occupancy assignment for both map representations
    if isnumeric(map) % the map is a numeric matrix
        map(cell(2), cell(1)) = value; % matrix occupancy using row y and column x
    else % Robotics System Toolbox occupancy-map assignment
        try % grid-coord assignment first
            setOccupancy(map, cell, value, "grid"); % occupancy map occupancy
        catch % for older MATLAB signatures
            setOccupancy(map, cell, value); % occupancy without the grid argument
        end % occupancy assignment try-catch
    end % representation branch
end % setCellOccupied

function [width, height] = getMapSize(map) % map-size lookup for both representations
    if isnumeric(map) % the map is a numeric matrix
        [height, width] = size(map); % matrix rows as height and columns as width
    else % Robotics System Toolbox metadata
        gridSize = map.GridSize; % occupancy map grid size
        width = gridSize(1); % number of x cells
        height = gridSize(2); % number of y cells
    end % representation branch
end % getMapSize

function occupancyMatrix = mapToMatrix(map) % conversion from map object to matrix
    [width, height] = getMapSize(map); % map dimensions
    occupancyMatrix = zeros(height, width); % output occupancy matrix
    for x = 1:width % every x coord
        for y = 1:height % every y coord
            occupancyMatrix(y, x) = double(isOccupiedCell(map, [x, y])); % occupancy value
        end % y loop
    end % x loop
end % mapToMatrix

function search = emptySearchStats(label) % default search-statistics structure
    search.cost = inf(1, 1); % dummy unreachable cost grid
    search.parentX = nan(1, 1); % dummy parent-x grid
    search.parentY = nan(1, 1); % dummy parent-y grid
    search.expanded = 0; % zero expanded nodes
    search.visited = 0; % zero visited nodes
    search.runtime = 0; % zero runtime
    search.label = label; % requested label
end % emptySearchStats

