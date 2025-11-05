function [start, goal] = selectStartGoal(mapData, img_display, mode, obstacleThreshold)
    % Display map for selecting start and goal points
    figure;
    set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
    imshow(img_display, []); colormap(gray); axis on;
    title('Click Start Point, then Goal Point');

    if nargin < 4
        obstacleThreshold = 0.5; % Default for occupancy maps
    end

    valid = false;
    while ~valid
        [x, y] = ginput(2); % User clicks two points

        % Coordinate handling based on mode
        if strcmp(mode, 'grid')
            start = round([y(1), x(1)]); % [row, col] for grid
            goal  = round([y(2), x(2)]);
        elseif strcmp(mode, 'occupancy')
            start = [x(1), y(1)];        % [x, y] for occupancy maps
            goal  = [x(2), y(2)];
        else
            error('Invalid mode. Use ''grid'' or ''occupancy''.');
        end

        % Validation
        if strcmp(mode, 'grid')
            % Grid-based validation (A*/Dijkstra)
            if mapData(start(1), start(2)) == 1
                disp('Start point is inside an obstacle. Please select again.');
            elseif mapData(goal(1), goal(2)) == 1
                disp('Goal point is inside an obstacle. Please select again.');
            else
                valid = true;
            end

        elseif strcmp(mode, 'occupancy')
            % Occupancy map validation (RRT/Potential Fields)
            if getOccupancy(mapData, start) > obstacleThreshold
                disp('Start point is inside an obstacle. Please select again.');
            elseif getOccupancy(mapData, goal) > obstacleThreshold
                disp('Goal point is inside an obstacle. Please select again.');
            else
                valid = true;
            end
        end
    end
end
