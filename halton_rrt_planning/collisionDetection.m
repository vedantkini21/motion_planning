% Halton RRT Planning
function inCollision = collisionDetection(polyA, polyB) % custom polygon collision detection for two CCW polygons
inCollision = false; % collision flag as false
if isempty(polyA) || isempty(polyB) % either polygon is empty
    return; % empty polygons cannot collide in this assignment model
end % empty polygon check
countA = size(polyA, 1); % count vertices in the first polygon
countB = size(polyB, 1); % count vertices in the second polygon
for i = 1:countA % every edge of polygon A
    a1 = polyA(i, :); % first endpoint of edge A
    a2 = polyA(wrapIndex(i + 1, countA), :); % second endpoint of edge A
    for j = 1:countB % every edge of polygon B
        b1 = polyB(j, :); % first endpoint of edge B
        b2 = polyB(wrapIndex(j + 1, countB), :); % second endpoint of edge B
        if segmentsIntersect(a1, a2, b1, b2) % the two polygon edges intersect
            inCollision = true; % polygons as colliding
            return; % immediately after detecting collision
        end % segment-intersection check
    end % polygon B edge loop
end % polygon A edge loop
if pointInPolygon(polyA(1, :), polyB) % polygon A lies inside polygon B
    inCollision = true; % containment as collision
    return; % after detecting containment
end % a-in-B containment check
if pointInPolygon(polyB(1, :), polyA) % polygon B lies inside polygon A
    inCollision = true; % containment as collision
    return; % after detecting containment
end % b-in-A containment check
end % collisionDetection

function idx = wrapIndex(indexValue, countValue) % one-based cyclic indexing
idx = mod(indexValue - 1, countValue) + 1; % wrap the index into the range one through countValue
end % wrapIndex

function hit = segmentsIntersect(a1, a2, b1, b2) % custom closed-segment intersection
o1 = orientationValue(a1, a2, b1); % orientation of a1-a2 with b1
o2 = orientationValue(a1, a2, b2); % orientation of a1-a2 with b2
o3 = orientationValue(b1, b2, a1); % orientation of b1-b2 with a1
o4 = orientationValue(b1, b2, a2); % orientation of b1-b2 with a2
hit = false; % intersection flag
if o1 * o2 < 0 && o3 * o4 < 0 % the strict crossing case
    hit = true; % strict crossing as intersection
    return; % after detecting intersection
end % strict crossing check
if abs(o1) < 1e-10 && pointOnSegment(b1, a1, a2) % b1 lies on segment a
    hit = true; % endpoint or collinear overlap as intersection
    return; % after detecting intersection
end % b1-on-a check
if abs(o2) < 1e-10 && pointOnSegment(b2, a1, a2) % b2 lies on segment a
    hit = true; % endpoint or collinear overlap as intersection
    return; % after detecting intersection
end % b2-on-a check
if abs(o3) < 1e-10 && pointOnSegment(a1, b1, b2) % a1 lies on segment b
    hit = true; % endpoint or collinear overlap as intersection
    return; % after detecting intersection
end % a1-on-b check
if abs(o4) < 1e-10 && pointOnSegment(a2, b1, b2) % a2 lies on segment b
    hit = true; % endpoint or collinear overlap as intersection
end % a2-on-b check
end % segmentsIntersect

function value = orientationValue(p, q, r) % signed area orientation test
value = (q(1) - p(1)) * (r(2) - p(2)) - (q(2) - p(2)) * (r(1) - p(1)); % two-dimensional cross product
end % orientationValue

function onSegment = pointOnSegment(point, edgeStart, edgeEnd) % endpoint-inclusive segment membership
insideX = point(1) >= min(edgeStart(1), edgeEnd(1)) - 1e-10 && point(1) <= max(edgeStart(1), edgeEnd(1)) + 1e-10; % x bounds with tolerance
insideY = point(2) >= min(edgeStart(2), edgeEnd(2)) - 1e-10 && point(2) <= max(edgeStart(2), edgeEnd(2)) + 1e-10; % y bounds with tolerance
onSegment = insideX && insideY; % coord bounds after collinearity has been checked
end % pointOnSegment

function inside = pointInPolygon(point, polygon) % ray-casting containment for non-convex polygons
inside = false; % inside flag
vertexCount = size(polygon, 1); % count polygon vertices
for i = 1:vertexCount % each polygon edge
    vertexA = polygon(i, :); % current edge start
    vertexB = polygon(wrapIndex(i + 1, vertexCount), :); % current edge end
    if abs(orientationValue(vertexA, vertexB, point)) < 1e-10 && pointOnSegment(point, vertexA, vertexB) % the point lies exactly on an edge
        inside = true; % boundary contact as inside
        return; % after boundary detection
    end % boundary check
    crossesRay = (vertexA(2) > point(2)) ~= (vertexB(2) > point(2)); % the horizontal ray crosses the edge's y-span
    if crossesRay % only for candidate ray crossings
        xAtRay = vertexA(1) + (point(2) - vertexA(2)) * (vertexB(1) - vertexA(1)) / (vertexB(2) - vertexA(2)); % edge intersection with the horizontal ray
        if xAtRay >= point(1) % the crossing is to the right of the point
            inside = ~inside; % the parity flag
        end % right-crossing check
    end % candidate-crossing branch
end % polygon edge loop
end % pointInPolygon

