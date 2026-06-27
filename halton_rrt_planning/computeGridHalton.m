% Halton RRT Planning
function points = computeGridHalton(N, p1, p2) % required Halton sequence generator
points = zeros(N, 2); % one row per sample and two coords per row
for k = 1:N % the number of Halton entries
    points(k, 1) = 100 * radicalInverse(k, p1); % scale the first radical inverse to the workspace x range
    points(k, 2) = 100 * radicalInverse(k, p2); % scale the second radical inverse to the workspace y range
end % halton sample loop
end % computeGridHalton

function value = radicalInverse(index, baseValue) % one-dimensional radical inverse helper
value = 0; % radical inverse value
factor = 1 / baseValue; % first digit weight
while index > 0 % while the integer still has base digits
    digit = mod(index, baseValue); % the least significant digit in the base
    value = value + digit * factor; % reflected digit contribution
    index = floor(index / baseValue); % the integer right in the base
    factor = factor / baseValue; % to the next reflected digit weight
end % digit-reflection loop
end % radicalInverse

