function dyn = normalizeDynObstacles(src)
% Normalize UI obstacle data (struct / cell / table / matrix) -> struct array
% Output fields: start [x y], end [x y], size, speed, pos [x y], hPatch []

    if isempty(src)
        dyn = []; return;
    end

    % Case 1: cell array of structs -> flatten
    if iscell(src) && ~isempty(src) && all(cellfun(@isstruct, src))
        dyn = [src{:}];
        dyn = ensureDynFields(dyn);
        return;
    end

    % Case 2: struct array -> ensure fields exist
    if isstruct(src)
        dyn = ensureDynFields(src);
        return;
    end

    % Case 3: table/cell/numeric -> coerce to numeric matrix
    if istable(src)
        M = table2array(src);
    elseif iscell(src)
        try
            M = cellfun(@str2double, src);  % works if strings; may yield NaN
            if any(isnan(M(:))), M = cell2mat(src); end
        catch
            M = cell2mat(src);
        end
    else
        M = src;  % numeric
    end

    if isempty(M)
        dyn = []; return;
    end

    % Expect columns: [sx sy ex ey size speed]
    n = size(M,1);
    tmpl = struct('start',[0 0],'end',[0 0],'size',6,'speed',1,'pos',[0 0],'hPatch',[]);
    dyn  = repmat(tmpl, 1, n);
    for i = 1:n
        sx = M(i,1); sy = M(i,2);
        ex = M(i,3); ey = M(i,4);
        sz = M(i,5); sp = M(i,6);
        dyn(i).start = [sx sy];
        dyn(i).end   = [ex ey];
        dyn(i).size  = sz;
        dyn(i).speed = sp;
        dyn(i).pos   = [sx sy];   % initial pos at start
        dyn(i).hPatch= [];
    end
end