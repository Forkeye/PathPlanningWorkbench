function rec = startRecorder(ax, enableFlag, params)
    % Initialize recorder state
    rec = struct('enabled',false, 'format','MP4', 'fps',30, ...
                 'path','', 'writer',[], 'first',true, ...
                 'useGetframe',true, 'i',0, 'step',1, ...
                 'W',0, 'H',0);   % target frame size (locked on first frame)

    if ~enableFlag, return; end

    % Options
    try rec.format = upper(params.record.format); catch, end
    try rec.fps    = params.record.fps;          catch, end

    % Prefer getframe (fast). If it throws on uiaxes, fallback to exportgraphics.
    try
        frame = getframe(ax); %#ok<NASGU>
        rec.useGetframe = true;
    catch
        rec.useGetframe = false;
        rec.step = 3;  % decimate exportgraphics captures to keep UI smooth
    end

    % Temp filename; Export... will copy/rename later
    ts = datestr(now,'yyyymmdd_HHMMSS');
    switch rec.format
        case 'MP4'
            rec.path = fullfile(tempdir, ['run_' ts '.mp4']);
            v = VideoWriter(rec.path, 'MPEG-4');
            v.FrameRate = rec.fps;
            open(v);
            rec.writer = v;
            rec.enabled = true;

        case 'GIF'
            rec.path = fullfile(tempdir, ['run_' ts '.gif']);
            rec.enabled = true;

        otherwise
            rec.enabled = false;
    end

    % Prime with one capture so we lock size immediately
    if rec.enabled
        recordFrame(rec, ax);
    end
end