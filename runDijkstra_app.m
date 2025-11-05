function R = runDijkstra_app(mapFile, startXY, goalXY, params, ax)

R = struct('success',false,'pathXY',[],'totalTime',NaN,'algorithmTime',NaN,'pathLen',NaN,'smoothness',NaN, ...
           'meta',struct('handles',struct('hRobot',[],'hPath',[],'hDyn',{cell(1,0)})));

% ---------- Load and preprocess (grid) ----------
% By default, process and display the same file
procFile = mapFile;
dispFile = mapFile;

% If the app asked for an alternate processing file, use it
useOverride = getfielddef(params,'useProcessingOverride',false);
if useOverride
    altProvided = getfielddef(params,'processingMapFile','');
    if ~isempty(altProvided) && exist(altProvided,'file')
        procFile = altProvided;   % process on hi-detail map
    else
        warning('Processing override requested but file not found: %s', altProvided);
    end
end

% Preprocess: grid from procFile; display image from dispFile
[grid, img_display, rows, cols] = preprocessMap(procFile, 'grid');

% Re-load display image to match the shown map (if different)
if ~strcmp(procFile, dispFile)
    [~, img_display, ~, ~] = preprocessMap(dispFile, 'grid');
end

% ---------- Convert Start/Goal from XY to RC ----------
start = round([startXY(2), startXY(1)]); % [row col]
goal  = round([goalXY(2),  goalXY(1)]);

% ---------- Inflate obstacles ----------
robotSize  = getfielddef(params,'robotSize',6);
halfSide   = robotSize/2;
robotShape = getfielddef(params,'robotShape','square');

isDisk = strcmpi(robotShape, 'disk');
if isDisk
    circleTheta = linspace(0, 2*pi, 50);   % reuse for every frame
else
    circleTheta = [];                      % not used
end

grid_inflated = inflateObstacles(grid, robotSize, 'grid', robotShape);

% Ensure start/goal are free
if grid_inflated(start(1), start(2)) ~= 0 || grid_inflated(goal(1), goal(2)) ~= 0
    % Try inverted interpretation
    grid_alt = ~logical(grid);
    grid_infl_alt = inflateObstacles(grid_alt, robotSize, 'grid', robotShape);
    if grid_infl_alt(start(1), start(2)) == 0 && grid_infl_alt(goal(1), goal(2)) == 0
        grid_inflated = grid_infl_alt;
        disp('Note: grid looked inverted; using inverted occupancy (0 = free, 1 = obstacle).');
    end
end

%% ---------- Costs & Heuristic (precomputed matrix) ----------
cost_straight  = 10;
cost_diagonal  = 14;

% ---------- Dijkstra: Moves ----------
moves = [ -1,  0, cost_straight;
           1,  0, cost_straight;
           0, -1, cost_straight;
           0,  1, cost_straight;
          -1, -1, cost_diagonal;
          -1,  1, cost_diagonal;
           1, -1, cost_diagonal;
           1,  1, cost_diagonal ];

% ---------- Dijkstra: Initial plan ----------
tPlanStart = tic;
path = planPath_local(start, goal, grid_inflated, rows, cols, moves);
planTime = toc(tPlanStart);

algorithmTime = planTime;

%% ---------- Dijkstra: Visualization setup on provided axes ----------
[hRobot, hPathLine] = plotPath_onAxes(ax, img_display, start, goal, path, ...
    "Dijkstra's algorithm", 'grid', [], halfSide, params.robotColor, params.robotShape);

% --- Start recorder (video) ---
rec = startRecorder(ax, isfield(params,'record') && params.record.enable, params);

% Return handles to the app (so Reset can delete them)
R.meta.handles.hRobot = hRobot;
R.meta.handles.hPath  = hPathLine;
R.meta.handles.hDyn   = {};

if isempty(path)
    % Metrics (same logic as script when failure)
    R.success = false;
    R.totalTime = planTime;
    R.algorithmTime = planTime;
    R.pathXY = [];
    R.pathLen = inf;
    R.smoothness = inf;
    return;
end

%% ---------- Dijkstra: Sensor & dynamic obstacle settings ----------
scan_radius    = 20;
arc_resolution = 20;
arc_half_angle = deg2rad(70);

% ---------- Dynamic Obstacles setup ----------
if isfield(params,'dynEnabled') && params.dynEnabled && ...
   isfield(params,'dynamicObstaclesUI') && ~isempty(params.dynamicObstaclesUI)

    % Normalize UI data -> struct array with required fields
    dynamicObstacles = normalizeDynObstacles(params.dynamicObstaclesUI);

    axes(ax); hold(ax,'on');
    for i = 1:numel(dynamicObstacles)
        dynamicObstacles(i).hPatch = createOrUpdateDynPatch( ...
            ax, dynamicObstacles(i).pos, dynamicObstacles(i).size, [], 0.35, getfielddef(params,'dynPosIsXY',true));
    end
else
    dynamicObstacles = [];   % no dynamics
end


globalFrame       = 1;

% State flags
avoid_mode        = false;
avoid_counter     = 0;
replan_requested  = false;
k                 = 1;
x = path(1,2);    % col
y = path(1,1);    % row

REACTION_THRESHOLD = 11;
escape_step_size   = 5;
alternate_escape   = false;
escape_attempts    = 0;


% ---------- Dijkstra: Main animation loop (frontal scan, avoid, replan) ----------
tAllStart = tic;

%% Make sure plotting targets this axes for any function using gca
axes(ax);
hold(ax,'on');

while true
    if ~isempty(dynamicObstacles)
        [sensor_grid, dynamicObstacles] = updateDynamicObstacles( ...
            dynamicObstacles, grid_inflated, params.robotSize, params.robotShape, rows, cols, globalFrame, 'grid');

        % Refresh patches
        for i = 1:numel(dynamicObstacles)
            dynamicObstacles(i).hPatch = createOrUpdateDynPatch( ...
                ax, dynamicObstacles(i).pos, dynamicObstacles(i).size, dynamicObstacles(i).hPatch, 0.35, params.dynPosIsXY);
        end
    else
        sensor_grid = grid_inflated;  % static only
    end
    
    if replan_requested
        disp('Obstacle in the way. Replanning path...');
        new_start = round([y, x]);
        tRe = tic;
        path = planPath_local(new_start, goal, sensor_grid, rows, cols, moves);
        rePlanTime = toc(tRe);
        algorithmTime = algorithmTime + rePlanTime;
        if isempty(path)
            disp('Replan failed: no path.');
            break;
        end

        if isvalidgfx(hPathLine), delete(hPathLine); end
        hPathLine = plot(ax, path(:,2), path(:,1), 'b-', 'LineWidth', 2, ...
            'HitTest','off','PickableParts','none');

        R.meta.handles.hPath = hPathLine; % keep latest handle
        k = 1;
        replan_requested = false;
        disp('New path found!');
        drawnow limitrate;
        continue;
    end

    if ~avoid_mode && k >= size(path,1)
        % Reached goal
        break;
    end

    if ~avoid_mode
        target   = path(k+1,:);
        path_dir = target - [y, x];
        if norm(path_dir) > 0
            path_dir = path_dir / norm(path_dir);
        end

        % ---- Frontal scan ----
        heading_angle = atan2(path_dir(1), path_dir(2));
        arc_angles = linspace(-arc_half_angle, arc_half_angle, arc_resolution) + heading_angle;
        obstacle_count = 0; left_hits = 0; center_hits = 0; right_hits = 0;

        for i = 1:length(arc_angles)
            a  = arc_angles(i);
            dx = round(scan_radius * cos(a));
            dy = round(scan_radius * sin(a));
            px = round(x + dx); % col
            py = round(y + dy); % row
            if px >= 1 && px <= cols && py >= 1 && py <= rows
                if sensor_grid(py, px) == 2
                    obstacle_count = obstacle_count + 1;
                    if i <= arc_resolution / 3
                        left_hits = left_hits + 1;
                    elseif i > arc_resolution * 2 / 3
                        right_hits = right_hits + 1;
                    else
                        center_hits = center_hits + 1;
                    end
                end
            end
        end

        if obstacle_count >= REACTION_THRESHOLD
            % Drop path, enter avoidance
            if center_hits >= left_hits && center_hits >= right_hits
                avoid_dir = [-path_dir(1), path_dir(2)];
            elseif left_hits > right_hits
                avoid_dir = [path_dir(2), -path_dir(1)];
            else
                avoid_dir = [-path_dir(2), path_dir(1)];
            end
            avoid_dir = avoid_dir / norm(avoid_dir);
            avoid_mode = true;
            avoid_counter = 0;
            path = [];

            if isvalidgfx(hPathLine)
                delete(hPathLine);
                hPathLine = [];
		R.meta.handles.hPath=[];
            end
            drawnow limitrate;
            continue;
        end

        % Follow original path (advance along path_dir in (row,col) space)
        x = x + path_dir(2);
        y = y + path_dir(1);

        if norm([y, x] - target) < 1
            k = k + 1;
        end

    else
        % ---- Avoidance mode ----
        next_x = x + avoid_dir(2);
        next_y = y + avoid_dir(1);
        ix = round(next_x); iy = round(next_y);
        if ix >= 1 && ix <= cols && iy >= 1 && iy <= rows && grid_inflated(iy, ix) == 0
            x = next_x;
            y = next_y;
        else
            disp('Avoidance step blocked — holding position');
        end

        avoid_counter = avoid_counter + 1;

        if avoid_counter >= 5
            goal_vec = goal - round([y, x]);    % in [row col]
            nrm = norm(goal_vec);
            if nrm > 0, goal_vec = goal_vec / nrm; end

            if ~alternate_escape
                escape_vec = [goal_vec(2), -goal_vec(1)];
            else
                escape_vec = [-goal_vec(2), goal_vec(1)];
            end

            for step = 1:escape_step_size
                test_x = round(x + escape_vec(2));
                test_y = round(y + escape_vec(1));
                if test_x < 1 || test_x > cols || test_y < 1 || test_y > rows || sensor_grid(test_y, test_x) ~= 0
                    disp('Blocked during animated sidestep — stopping early');
                    break;
                end
                x = test_x;
                y = test_y;

                % Animate robot at each sidestep
                if isDisk
                    cx = x + halfSide * cos(circleTheta);
                    cy = y + halfSide * sin(circleTheta);
                else
                    cx = [x-halfSide, x+halfSide, x+halfSide, x-halfSide];
                    cy = [y-halfSide, y-halfSide, y+halfSide, y+halfSide];
                end
                set(hRobot, 'XData', cx, 'YData', cy);
                drawnow limitrate;
                pause(0.02);
            end
            disp(['Blocked — sidestepped ', num2str(step), ' steps.']);

            alternate_escape = ~alternate_escape;
            escape_attempts = escape_attempts + 1;
            if mod(escape_attempts, 2) == 0
                escape_step_size = escape_step_size * 2;
            end

            iy = round(y); ix = round(x);
            if grid_inflated(iy, ix) ~= 0
                disp('Robot ended up in obstacle — correcting position.');
                avoid_mode = true;
                avoid_counter = 0;
                drawnow limitrate;
                continue;
            end

            avoid_mode = false;
            replan_requested = true;
        end
    end

    % Animate robot footprint
    if isDisk
        cx = x + halfSide * cos(circleTheta);
        cy = y + halfSide * sin(circleTheta);
    else
        cx = [x-halfSide, x+halfSide, x+halfSide, x-halfSide];
        cy = [y-halfSide, y-halfSide, y+halfSide, y+halfSide];
    end
    set(hRobot, 'XData', cx, 'YData', cy);

    drawnow limitrate;
    pause(0.02);

    recordFrame(rec, ax);
    
    globalFrame = globalFrame + 1;
end

stopRecorder(rec);
R.meta.video = struct('enabled',rec.enabled, 'path', rec.path, 'format', rec.format);

hold(ax,'off');

%% ---------- Dijkstra: Performance Metrics ----------
R.success = ~isempty(path);
R.totalTime    = toc(tAllStart);
R.algorithmTime = algorithmTime;
if R.success
    diffs = diff(path);
    step_lengths = sqrt(sum(diffs.^2, 2));
    pathLength = sum(step_lengths);

    directions = atan2(diffs(:,1), diffs(:,2));
    angle_changes = diff(directions);
    angle_changes = mod(angle_changes + pi, 2*pi) - pi;
    pathSmoothness = sum(abs(angle_changes));

    R.pathXY     = [path(:,2), path(:,1)]; % to [x y]
    R.pathLen    = pathLength;
    R.smoothness = pathSmoothness;
else
    R.pathXY     = [];
    R.pathLen    = inf;
    R.smoothness = inf;
end

% keep obstacle handles for the app to clear
%R.meta.handles.hDyn = arrayfun(@(d)d.hPatch, dynamicObstacles, 'UniformOutput', false);
if ~isempty(dynamicObstacles)
    R.meta.handles.hDyn = {dynamicObstacles.hPatch};
else
    R.meta.handles.hDyn = {};
end

end % runDijkstra_app


%% ========================= local helpers =========================

function val = getfielddef(s, f, d)
    if isfield(s,f) && ~isempty(s.(f)), val = s.(f); else, val = d; end
end

function tf = isvalidgfx(h)
    tf = ~isempty(h) && isgraphics(h) && isvalid(h);
end

function path = planPath_local(start, goal, map, rows, cols, moves)
    tic;
    % Preallocate cost and tracking matrices
    costs = inf(rows, cols);
    parent = cell(rows, cols);
    visited = false(rows, cols);
    openList = false(rows, cols); % Tracks if node is in heap

    % Initialize costs
    costs(start(1), start(2)) = 0;

    % Min-heap: [cost, row, col]
    heap = [0, start(1), start(2)];
    openList(start(1), start(2)) = true;

    while true
        % If heap is empty, no path exists
        if isempty(heap)
            path = [];
            return;
        end

        % Extract node with minimum cost
        [~, minIdx] = min(heap(:,1));
        current = heap(minIdx, :);
        heap(minIdx,:) = [];  % remove from heap

        r = current(2);
        c = current(3);

        % Mark node as visited and remove from open list
        visited(r, c) = true;
        openList(r, c) = false;

        % Check if goal is reached
        if r == goal(1) && c == goal(2)
            break;
        end

        % Explore neighbors
        for i = 1:size(moves, 1)
            nr = r + moves(i,1);
            nc = c + moves(i,2);
            mc = moves(i,3);

            if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
                if map(nr, nc) == 0 && ~visited(nr, nc)
                    newCost = costs(r,c) + mc;
                    if newCost < costs(nr,nc)
                        costs(nr,nc) = newCost;
                        parent{nr,nc} = [r,c];

                        if ~openList(nr,nc)
                            heap = [heap; newCost, nr, nc];
                            openList(nr,nc) = true;
                        else
                            % Update cost if already present
                            idx = find(heap(:,2) == nr & heap(:,3) == nc, 1);
                            if ~isempty(idx)
                                heap(idx, 1) = newCost;
                            end
                        end
                    end
                end
            end
        end
    end

    % Reconstruct path
    path = goal;
    while ~isequal(path(1,:), start)
        p = parent{path(1,1), path(1,2)};
        if isempty(p), path = []; return; end
        path = [p; path];
    end
    elapsedTime = toc;
    fprintf('Time Taken    : %.4f sec\n', elapsedTime);
end

% Testing
% function path = planPath_local(start, goal, map, rows, cols, moves)
%     tic;
%     % Preallocate cost and tracking matrices
%     costs = inf(rows, cols);
%     parent = cell(rows, cols);
%     visited = false(rows, cols);
%     heapPos = zeros(rows, cols); % Tracks position of node in heap (0 if not in heap)
% 
%     % Initialize costs
%     costs(start(1), start(2)) = 0;
% 
%     % Min-heap: [cost, row, col]
%     heap = [0, start(1), start(2)];
%     heapPos(start(1), start(2)) = 1;
%     heapSize = 1;
% 
%     while heapSize > 0
%         % Extract node with minimum cost (pop from heap)
%         current = heap(1, :);
%         r = current(2);
%         c = current(3);
% 
%         % Remove root and restore heap property
%         heap(1, :) = heap(heapSize, :);
%         if heapSize > 1
%             heapPos(heap(1,2), heap(1,3)) = 1;
%         end
%         heapSize = heapSize - 1;
%         heapPos(r, c) = 0;
% 
%         if heapSize > 0
%             heapifyDown(1);
%         end
% 
%         % Mark node as visited
%         visited(r, c) = true;
% 
%         % Check if goal is reached
%         if r == goal(1) && c == goal(2)
%             break;
%         end
% 
%         % Explore neighbors
%         for i = 1:size(moves, 1)
%             nr = r + moves(i,1);
%             nc = c + moves(i,2);
%             mc = moves(i,3);
% 
%             if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
%                 if map(nr, nc) == 0 && ~visited(nr, nc)
%                     newCost = costs(r,c) + mc;
% 
%                     if newCost < costs(nr,nc)
%                         costs(nr,nc) = newCost;
%                         parent{nr,nc} = [r,c];
% 
%                         if heapPos(nr,nc) == 0
%                             % Insert new node into heap
%                             heapSize = heapSize + 1;
%                             heap(heapSize, :) = [newCost, nr, nc];
%                             heapPos(nr,nc) = heapSize;
%                             heapifyUp(heapSize);
%                         else
%                             % Update existing node in heap
%                             idx = heapPos(nr,nc);
%                             heap(idx, 1) = newCost;
%                             heapifyUp(idx);
%                         end
%                     end
%                 end
%             end
%         end
%     end
% 
%     % Reconstruct path
%     if ~visited(goal(1), goal(2))
%         path = [];
%         elapsedTime = toc;
%         fprintf('Time Taken    : %.4f sec (No path found)\n', elapsedTime);
%         return;
%     end
% 
%     path = goal;
%     while ~isequal(path(1,:), start)
%         p = parent{path(1,1), path(1,2)};
%         if isempty(p), path = []; return; end
%         path = [p; path];
%     end
% 
%     elapsedTime = toc;
%     fprintf('Time Taken    : %.4f sec\n', elapsedTime);
% 
%     % Nested function: Bubble up to maintain min-heap property
%     function heapifyUp(idx)
%         while idx > 1
%             parentIdx = floor(idx / 2);
%             if heap(idx, 1) < heap(parentIdx, 1)
%                 % Swap
%                 temp = heap(idx, :);
%                 heap(idx, :) = heap(parentIdx, :);
%                 heap(parentIdx, :) = temp;
% 
%                 % Update position tracking
%                 heapPos(heap(idx,2), heap(idx,3)) = idx;
%                 heapPos(heap(parentIdx,2), heap(parentIdx,3)) = parentIdx;
% 
%                 idx = parentIdx;
%             else
%                 break;
%             end
%         end
%     end
% 
%     % Nested function: Bubble down to maintain min-heap property
%     function heapifyDown(idx)
%         while true
%             leftChild = 2 * idx;
%             rightChild = 2 * idx + 1;
%             smallest = idx;
% 
%             if leftChild <= heapSize && heap(leftChild, 1) < heap(smallest, 1)
%                 smallest = leftChild;
%             end
% 
%             if rightChild <= heapSize && heap(rightChild, 1) < heap(smallest, 1)
%                 smallest = rightChild;
%             end
% 
%             if smallest ~= idx
%                 % Swap
%                 temp = heap(idx, :);
%                 heap(idx, :) = heap(smallest, :);
%                 heap(smallest, :) = temp;
% 
%                 % Update position tracking
%                 heapPos(heap(idx,2), heap(idx,3)) = idx;
%                 heapPos(heap(smallest,2), heap(smallest,3)) = smallest;
% 
%                 idx = smallest;
%             else
%                 break;
%             end
%         end
%     end
% end