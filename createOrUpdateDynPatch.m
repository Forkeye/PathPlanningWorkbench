function h = createOrUpdateDynPatch(ax, pos, sizePix, h, alpha, posIsXY)
    if nargin < 6, posIsXY = false; end       % default: pos = [row col]
    c  = pos; 
    s  = sizePix; 
    hs = s/2;

    % Map pos -> display coords
    if posIsXY
        x0 = c(1);  y0 = c(2);                 % pos is [x y]
    else
        x0 = c(2);  y0 = c(1);                 % pos is [row col] -> [x y]
    end

    x = [x0-hs, x0+hs, x0+hs, x0-hs];
    y = [y0-hs, y0-hs, y0+hs, y0+hs];          % <- fixed: y0+hs

    if ~isvalidgfx(h)
        h = patch(ax, 'XData', x, 'YData', y, ...
                  'FaceColor',[1 0 0], 'EdgeColor',[1 0 0], ...
                  'FaceAlpha', alpha, 'LineWidth', 1.0, ...
                  'HitTest','off','PickableParts','none');
    else
        set(h, 'XData', x, 'YData', y);
    end
end