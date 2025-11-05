function [hRobot, hPathLine] = plotPath_onAxes(ax, img_display, startRC, ...
    goalRC, pathRC, titleStr, ~, treeOrEdges, halfSide, robotColor, robotShape)
cla(ax);
imshow(img_display,'Parent',ax);
axis(ax,'image'); title(ax,titleStr,'Parent',ax);
hImg = findobj(ax,'Type','Image'); set(hImg,'HitTest','off','PickableParts','none');
hold(ax,'on');

if exist('treeOrEdges','var') && ~isempty(treeOrEdges)
    if isstruct(treeOrEdges) && isfield(treeOrEdges,'pos') && isfield(treeOrEdges,'parent')
        % struct tree -> draw edges by parent linkage (convert RC->XY)
        for ii = 2:numel(treeOrEdges)
            p = treeOrEdges(ii).parent;
            if p > 0
                p1 = treeOrEdges(ii).pos;    % [row col]
                p2 = treeOrEdges(p).pos;     % [row col]
                plot(ax, [p1(2) p2(2)], [p1(1) p2(1)], 'Color',[0.2 0.8 0.2], ...
                     'LineWidth', 0.75, 'HitTest','off','PickableParts','none');
            end
        end
    elseif isnumeric(treeOrEdges) && size(treeOrEdges,2) == 4
        % Nx4 segments [x1 y1 x2 y2]
        line(ax, [treeOrEdges(:,1) treeOrEdges(:,3)]', ...
                 [treeOrEdges(:,2) treeOrEdges(:,4)]', ...
                 'Color',[0.2 0.8 0.2], 'LineWidth',0.75, ...
                 'HitTest','off','PickableParts','none');
    end
end

% Start/Goal markers (convert RC->XY for display)
startXY = [startRC(2), startRC(1)];
goalXY  = [goalRC(2),  goalRC(1)];
plot(ax, startXY(1), startXY(2), 'g*','MarkerSize',10,'HitTest','off','PickableParts','none');
plot(ax, goalXY(1),  goalXY(2),  'rs','MarkerSize',8, 'HitTest','off','PickableParts','none');

if ~exist('robotColor','var') || isempty(robotColor)
    robotColor = [0 1 1];
end

% Path (if present)
if ~isempty(pathRC)
    Pxy = [pathRC(:,2), pathRC(:,1)];
    hPathLine = line(ax, Pxy(:,1), Pxy(:,2), 'LineWidth',2, 'Color',[0 0.4 1], ...
                     'HitTest','off','PickableParts','none');
else
    hPathLine = line(ax, NaN, NaN, 'LineWidth',2, 'Color',[0 0.4 1], ...
                     'HitTest','off','PickableParts','none');
end

% Robot footprint
if strcmpi(robotShape,"square")
    cx = [startXY(1)-halfSide, startXY(1)+halfSide, startXY(1)+halfSide, startXY(1)-halfSide];
    cy = [startXY(2)-halfSide, startXY(2)-halfSide, startXY(2)+halfSide, startXY(2)+halfSide];
    hRobot = patch(ax, cx, cy, robotColor, ...
                   'EdgeColor','k','LineWidth',1.0, ...
                   'HitTest','off','PickableParts','none');

elseif strcmpi(robotShape,"disk")
    theta = linspace(0,2*pi,50);
    cx = startXY(1) + halfSide*cos(theta);
    cy = startXY(2) + halfSide*sin(theta);
    hRobot = patch(ax, cx, cy, robotColor, ...
                   'EdgeColor','k','LineWidth',1.0, ...
                   'HitTest','off','PickableParts','none');
else
    % fallback (just a cross)
    hRobot = plot(ax, startXY(1), startXY(2), 'x','Color',robotColor,'MarkerSize',8);
end

hold(ax,'off');
end