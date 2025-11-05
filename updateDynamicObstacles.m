function [sensor_map, dynamicObstacles] = updateDynamicObstacles( ...
    dynamicObstacles, base_map, robotSize, robotShape, rows, cols, globalFrame, ~, obstacleThreshold)

% 1) Structuring element for robot inflation
switch robotShape
    case 'square', se_robot = strel('square', robotSize);
    case 'disk',   se_robot = strel('disk',   robotSize);
    otherwise, error('Invalid robot shape. Use ''square'' or ''disk''.');
end

% 2) Initialize sensor grid from base_map
if nargin < 9 || isempty(obstacleThreshold), obstacleThreshold = 0.5; end
if isa(base_map, 'occupancyMap')
    occ = getOccupancy(base_map);
    sensor_matrix = double(occ > obstacleThreshold);
else
    sensor_matrix = double(base_map);   % << no getOccupancy on matrices
end


% 2) Stamp dynamic obstacles using image-style indexing (same for both modes)
for i = 1:numel(dynamicObstacles)
    [dynamicObstacles(i).hPatch, currentPos] = moveDynamicObstacle( ...
        dynamicObstacles(i).hPatch, ...
        dynamicObstacles(i).start, ...
        dynamicObstacles(i).end, ...
        dynamicObstacles(i).size, ...
        dynamicObstacles(i).speed, ...
        globalFrame);

    dynamicObstacles(i).pos = currentPos;

    halfO = round(dynamicObstacles(i).size / 2);
    x1 = max(1, currentPos(1) - halfO);
    x2 = min(cols, currentPos(1) + halfO);
    y1 = max(1, currentPos(2) - halfO);
    y2 = min(rows, currentPos(2) + halfO);

    if x1 <= x2 && y1 <= y2
        obstacle_patch = false(rows, cols);
        obstacle_patch(y1:y2, x1:x2) = true;          % image-style
        obstacle_patch = imdilate(obstacle_patch, se_robot);

        % Mark dynamic cells as 2 (scan looks for == 2)
        sensor_matrix(obstacle_patch) = 2;
    end
end

% Prepare sensor_map (same numeric grid back out)
sensor_map   = sensor_matrix;
end
