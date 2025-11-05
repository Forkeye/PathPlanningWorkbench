function recordFrame(rec, ax)
    if ~rec.enabled, return; end

    rec.i = rec.i + 1;
    if rec.i > 1 && ~rec.useGetframe && mod(rec.i, rec.step) ~= 0
        return; % decimate heavy exportgraphics captures
    end

    % Grab a frame
    if rec.useGetframe
        fr  = getframe(ax);
        img = frame2im(fr);
    else
        tmp = [tempname '.png'];
        exportgraphics(ax, tmp, 'BackgroundColor','current', 'ContentType','image');
        img = imread(tmp);
        delete(tmp);
    end

    % Convert to uint8 RGB if needed
    if ~isa(img,'uint8'), img = im2uint8(img); end
    if size(img,3) == 1,  img = repmat(img,[1 1 3]); end

    % On first frame: lock target size (even dimensions)
    [h,w,~] = size(img);
    if rec.first
        rec.H = h - mod(h,2);       % make even
        rec.W = w - mod(w,2);       % make even
        if rec.H < 2, rec.H = 2; end
        if rec.W < 2, rec.W = 2; end
        if strcmp(rec.format,'MP4')
            % MP4 writer will take the first frame's size; ensure we give that
            if rec.H ~= h || rec.W ~= w
                img = imresize(img, [rec.H rec.W]);
            end
        end
        rec.first = false;
    else
        % Subsequent frames: force to locked size
        if size(img,1) ~= rec.H || size(img,2) ~= rec.W
            img = imresize(img, [rec.H rec.W]);
        end
    end

    % Write
    switch rec.format
        case 'MP4'
            writeVideo(rec.writer, im2frame(img));
        case 'GIF'
            [imind, cm] = rgb2ind(img, 256);
            if rec.i == 1
                imwrite(imind, cm, rec.path, 'gif', 'LoopCount', inf, 'DelayTime', 1/rec.fps);
            else
                imwrite(imind, cm, rec.path, 'gif', 'WriteMode', 'append', 'DelayTime', 1/rec.fps);
            end
    end
end