function inflated_map = inflateObstacles(mapData, robotSize, mode, shape)
    % Default shape
    if nargin < 4
        shape = 'square';
    end

    % Create structuring element based on shape
    if strcmp(shape, 'square')
        se_robot = strel('square', round(robotSize/2)+2);
    elseif strcmp(shape, 'disk')
        se_robot = strel('disk', round(robotSize/2)+2);
    else
        error('Invalid shape. Use ''square'' or ''disk''.');
    end

    % Inflate based on map type
    if strcmp(mode, 'grid')
        % Binary grid inflation (A*, Dijkstra)
        % BW = mapData > 0;
        % bd = [BW(1,:), BW(end,:), BW(:,1).', BW(:,end).'];
        % if mean(bd(:)) > 0.5, BW = ~BW; end
        % inflated_map = imdilate(BW, se_robot);

        BW = mapData > 0;
        if mean(BW(:)) > 0.5, BW = ~BW; end
        BW = imdilate(BW, se_robot);
        inflated_map = cast(BW, 'like', mapData);


    elseif strcmp(mode, 'occupancy')
        % Occupancy map inflation (RRT, Potential Fields)
        occMatrix = getOccupancy(mapData) > 0.5;
        occMatrix = imdilate(occMatrix, se_robot);
        inflated_map = occupancyMap(occMatrix, 1);
    else
        error('Invalid mode. Use ''grid'' or ''occupancy''.');
    end
end
