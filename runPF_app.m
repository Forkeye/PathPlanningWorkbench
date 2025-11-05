function R = runPF_app(mapFile, startXY, goalXY, params, ax)

R = struct('success',false,'pathXY',[],'totalTime',NaN,'algorithmTime',NaN,'pathLen',NaN,'smoothness',NaN, ...
           'meta',struct('handles',struct('hRobot',[],'hPath',[],'hDyn',{cell(1,0)})));

% ---------- Load and preprocess (map) ----------
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
[map, img_display, rows, cols] = preprocessMap(procFile, 'grid');

% Re-load display image to match the shown map (if different)
if ~strcmp(procFile, dispFile)
    [~, img_display, ~, ~] = preprocessMap(dispFile, 'grid');
end

%% Convert Start/Goal from XY to RC
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
    circleTheta = [];                       % not used
end

map_inflated = inflateObstacles(map, robotSize, 'grid', robotShape);

% Ensure start/goal are free
if map_inflated(start(1), start(2)) ~= 0 || map_inflated(goal(1), goal(2)) ~= 0
    % Try inverted interpretation
    map_alt = ~logical(map);
    map_infl_alt = inflateObstacles(map_alt, robotSize, 'grid', robotShape);
    if map_infl_alt(start(1), start(2)) == 0 && map_infl_alt(goal(1), goal(2)) == 0
        map_inflated = map_infl_alt;
        disp('Note: map looked inverted; using inverted occupancy (0 = free, 1 = obstacle).');
    end
end

%% Fetch Params
k_att = getfielddef(params, 'pf_k_att', 1/70);
k_rep = getfielddef(params, 'pf_k_rep', 80);
Qstar = getfielddef(params, 'pf_Qstar', 2);
maxIts = getfielddef(params, 'pf_maxIts', 1000);
tol    = getfielddef(params, 'pf_tol', 1);
maxSmoothDist = getfielddef(params, 'pf_MaxSmoothDist', 30);

% robotColor = getfielddef(params,'robotColor',[0.4235 0.9569 1]);
% robotShape = getfielddef(params,'robotShape','square');
% robotSize  = getfielddef(params,'robotSize',6);

%% ---------- PT: Initial plan ----------
tPlanStart = tic;
path = planPath_local(start, goal, map_inflated, k_att, k_rep, Qstar, maxIts, tol);
planTime = toc(tPlanStart);

algorithmTime = planTime;

if ~isempty(path)
    path = smoothPathMaxDist(path, map_inflated, maxSmoothDist);
end

%% ---------- PT: Visualization setup on provided axes ----------
[hRobot, hPathLine] = plotPath_onAxes(ax, img_display, start, goal, path, ...
    'Potential Fields algorithm', 'grid', [], halfSide, params.robotColor, params.robotShape);

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

%% ---------- RRT: Sensor & dynamic obstacle settings ----------
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

progressCheckInterval = 100; % number of frames between progress checks
progressCounter = 0;
goal_xy        = [goal(2), goal(1)];
lastProgressDist = norm([x, y] - goal_xy);
progressThreshold = 1.0; % required improvement in distance to count as progress

stuck = false; % flag to mark if stuck

% ---------- RRT: Main animation loop (frontal scan, avoid, replan) ----------
tAllStart = tic;

%% Make sure plotting targets this axes for any function using gca
axes(ax);
hold(ax,'on');

while true
    if ~isempty(dynamicObstacles)
        [sensor_grid, dynamicObstacles] = updateDynamicObstacles( ...
            dynamicObstacles, map_inflated, params.robotSize, params.robotShape, rows, cols, globalFrame, 'grid');

        % Refresh patches
        for i = 1:numel(dynamicObstacles)
            dynamicObstacles(i).hPatch = createOrUpdateDynPatch( ...
                ax, dynamicObstacles(i).pos, dynamicObstacles(i).size, dynamicObstacles(i).hPatch, 0.35, params.dynPosIsXY);
        end
    else
        sensor_grid = map_inflated;     % ensure scan uses a defined matrix
    end
    
    if replan_requested
        disp('Obstacle in the way. Replanning path...');
        new_start = round([y, x]);
        tRe = tic;
        path = planPath_local(new_start, goal, sensor_grid, k_att, k_rep, Qstar, maxIts, tol);
        rePlanTime = toc(tRe);
        algorithmTime = algorithmTime + rePlanTime;
        if isempty(path)
            disp('Replan failed: no path.');
            break;
        else
            path = smoothPathMaxDist(path, sensor_grid, maxSmoothDist);
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
        target    = path(k+1,:);                % [row col]
        target_xy = [target(2), target(1)];     % -> [x y]
        path_dir  = target_xy - [x, y];
        if norm(path_dir) > 0
            path_dir = path_dir / norm(path_dir);
        end

        % ---- Frontal scan (RRT) ----
        heading_angle = atan2(path_dir(2), path_dir(1));
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
                avoid_dir = [-path_dir(2), path_dir(1)];
            elseif left_hits > right_hits
                avoid_dir = [path_dir(1), -path_dir(2)];
            else
                avoid_dir = [-path_dir(1), path_dir(2)];
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
        x = x + path_dir(1);
        y = y + path_dir(2);

        if norm([x, y] - target_xy) < 1
            k = k + 1;
        end

    else
        % ---- Avoidance mode ----
        next_x = x + avoid_dir(1);
        next_y = y + avoid_dir(2);
        ix = round(next_x); iy = round(next_y);
        if ix >= 1 && ix <= cols && iy >= 1 && iy <= rows && map_inflated(iy, ix) == 0
            x = next_x;
            y = next_y;
        else
            disp('Avoidance step blocked — holding position');
        end

        avoid_counter = avoid_counter + 1;

        if avoid_counter >= 5
            goal_xy  = [goal(2), goal(1)];      % [x y]
            goal_vec = goal_xy - [x, y];        % [x y]
            nrm = norm(goal_vec);
            if nrm > 0, goal_vec = goal_vec / nrm; end

            if ~alternate_escape
                % 90° left of goal_vec
                escape_vec = [-goal_vec(2), goal_vec(1)];
            else
                % 90° right of goal_vec
                escape_vec = [goal_vec(2), -goal_vec(1)];
            end

            for step = 1:escape_step_size
                test_x = round(x + escape_vec(1));
                test_y = round(y + escape_vec(2));
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
            if map_inflated(iy, ix) ~= 0
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



    progressCounter = progressCounter + 1;

    if progressCounter >= progressCheckInterval
        currentDist = norm([x, y] - goal_xy);
    
        if currentDist >= (lastProgressDist - progressThreshold)
            disp('Seems like the robot is stuck in local minima — terminating.');
            stuck = true; % mark stuck
            break; % exit while loop
        else
            % Progress was made → update lastProgressDist
            lastProgressDist = currentDist;
        end
    
        % Reset counter
        progressCounter = 0;
    end
end

stopRecorder(rec);
R.meta.video = struct('enabled',rec.enabled, 'path', rec.path, 'format', rec.format);

hold(ax,'off');

%% ---------- RRT: Performance Metrics ----------
R.success = ~isempty(path) && ~stuck;
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

end % runRRT_app


%% ========================= local helpers =========================
function val = getfielddef(s, f, d)
    if isfield(s,f) && ~isempty(s.(f)), val = s.(f); else, val = d; end
end

function tf = isvalidgfx(h)
    tf = ~isempty(h) && isgraphics(h) && isvalid(h);
end

% PT Path Planner Function
function [path] = planPath_local(start, goal, map, k_att, k_rep, Qstar, maxIts, tol)
    tic;

    % map is a numeric/grid matrix: 0 = free, nonzero = obstacle (incl. dynamic=2)
    if islogical(map)
        obstacle = map;
    else
        obstacle = map ~= 0;
    end

    % PT gains (tunable later via UI if you want)
    K    = 3;        % distance scaling for bwdist normalization
    d0   = Qstar;        % obstacle influence radius (px)
    Eta  = k_rep;       % repulsive gain
    zeta = k_att;     % attractive gain

    [repulsive, attractive] = computePotentials(obstacle, K, d0, Eta, zeta, goal);
    totalPotential = repulsive + attractive;

    [gR,gC] = gradient(totalPotential);
    s = start;  % [row col]
    vtest = -[gR(round(s(1)), round(s(2))), gC(round(s(1)), round(s(2)))];
    disp("dirTowardGoal? dot=" + dot(vtest, (goal - start)));

    path = gradientPlanner(totalPotential, start, goal, maxIts, tol);

    elapsedTime = toc;
    fprintf('Time Taken    : %.4f sec\n', elapsedTime);
end


% --- Compute Potentials ---
function [rep, att] = computePotentials(obstacle, K, d0, Eta, zeta, goalRC)
    % obstacle: logical/grid, RC orientation (no flip)
    % goalRC : [row col]

    % --- Distance Transform & Repulsive ---
    D   = bwdist(obstacle);
    Rho = max(D ./ K, eps);           % avoid division by zero

    rep = zeros(size(Rho));
    mask = Rho < d0;
    rep(mask) = Eta * (1 ./ Rho(mask) - 1/d0).^2;

    % --- Attractive (parabolic) ---
    [X, Y] = meshgrid(1:size(obstacle,2), 1:size(obstacle,1));  % X=cols, Y=rows
    r = hypot(Y - goalRC(1), X - goalRC(2));                    % RC-correct distance
    att = zeta * r.^2;

    % Zero inside a small goal radius
    goalRadius = max(2, round(0.01 * max(size(obstacle))));
    att(r < goalRadius) = 0;
end


% --- Gradient Descent Planner ---
function route = gradientPlanner(f, startRC, goalRC, maxIts, tol)
    % f: potential on a grid, RC orientation (rows, cols)

    % Ensure 2D double
    if ndims(f) ~= 2
        error('Potential field f must be 2D; got ndims(f)=%d', ndims(f));
    end
    f = double(f);

    [nRows, nCols] = size(f);

    % ----- Robust finite differences (no size mismatches) -----
    % d f / d row  (downwards is +)
    if nRows == 1
        gRow = zeros(1, nCols);
    else
        % forward on top, central in middle, backward on bottom
        top    =  (f(2,:) - f(1,:));
        middle =  0.5 * (f(3:end,:) - f(1:end-2,:));
        bottom =  (f(end,:) - f(end-1,:));
        gRow   = [top; middle; bottom];
    end

    % d f / d col  (rightwards is +)
    if nCols == 1
        gCol = zeros(nRows, 1);
    else
        % forward on left, central in middle, backward on right
        left   =  (f(:,2) - f(:,1));
        middle =  0.5 * (f(:,3:end) - f(:,1:end-2));
        right  =  (f(:,end) - f(:,end-1));
        gCol   = [left, middle, right];
    end

    route = startRC;
    pos   = startRC;  % [row col]

    % ---- DEBUG once: check initial direction aligns with goal ----
    iy0 = min(max(round(pos(1)),1), nRows);
    ix0 = min(max(round(pos(2)),1), nCols);
    v0  = -[gRow(iy0,ix0), gCol(iy0,ix0)];     % negative gradient (descent)
    dot0 = dot(v0, (goalRC - pos));
    fprintf('PF debug: start=%s goal=%s  v0=%s  dot(v0,goal-start)=%.4g\n', ...
        mat2str(pos), mat2str(goalRC), mat2str(v0,4), dot0);

    for i = 1:maxIts
        % Stop when close enough (RC metric)
        if norm(pos - goalRC) < tol
            route(end+1,:) = goalRC;
            break;
        end

        iy = min(max(round(pos(1)),1), nRows);
        ix = min(max(round(pos(2)),1), nCols);

        % Negative gradient: move to lower potential
        v = -[gRow(iy,ix), gCol(iy,ix)];

        % If flat (or near-flat), bias toward goal
        if norm(v) < 1e-12
            v = goalRC - pos;
        end
        v = v / (norm(v) + eps);

        % Adaptive step (same as before)
        alpha = max(0.5, min(3, norm(pos - goalRC) / 50));

        % Candidate next position (RC), clamp to grid
        next = pos + alpha * v;
        next(1) = max(1, min(next(1), nRows));  % row
        next(2) = max(1, min(next(2), nCols));  % col

        % Optional: prevent climbing uphill (tiny safeguard)
        % if f(round(next(1)), round(next(2))) > f(iy, ix)
        %     alpha = 0.5 * alpha;
        %     next = pos + alpha * v;
        %     next(1) = max(1, min(next(1), nRows));
        %     next(2) = max(1, min(next(2), nCols));
        % end

        pos = next;
        route(end+1,:) = pos;

        if i == 1
            fprintf('PF debug: first step v=%s  alpha=%.3g  next=%s\n', ...
                mat2str(v,4), alpha, mat2str(pos,3));
        end
    end
end

function path2 = smoothPathMaxDist(path, occ, maxSmoothDist)
% Greedy smoothing with distance gate; path is Nx2 [row col]; occ is logical grid
    if size(path,1) <= 2, path2 = path; return; end
    smoothed = path(1,:);
    i = 1;
    N = size(path,1);
    while i < N
        found = false;
        for j = N:-1:(i+1)
            if norm(path(i,:) - path(j,:)) <= maxSmoothDist
                if ~isCollision(occ, path(i,:), path(j,:))
                    smoothed = [smoothed; path(j,:)]; %#ok<AGROW>
                    i = j;
                    found = true;
                    break;
                end
            end
        end
        if ~found
            i = i + 1;
        end
    end
    path2 = smoothed;
end

% Collision check
function collision = isCollision(map, p1, p2)
    % Derive bounds locally (fixes "rows/cols undefined")
    [rows, cols] = size(map);

    baseResolution = 0.5;
    dist = norm(p2 - p1);

    % Dynamically adjust resolution for efficiency
    resolution = max(baseResolution, dist / 20); % at most 20 samples
    steps = ceil(dist / resolution);
    if steps < 2
        steps = 2; % Always sample start and end
    end

    % Interpolate along the segment in [row col]
    rr = linspace(p1(1), p2(1), steps);   % rows
    cc = linspace(p1(2), p2(2), steps);   % cols

    collision = false;
    for k = 1:steps
        r = round(rr(k)); 
        c = round(cc(k));
        % Treat any nonzero as obstacle; also guard bounds
        if r < 1 || r > rows || c < 1 || c > cols || map(r,c) ~= 0
            collision = true;
            return; % Early exit on collision
        end
    end
end
