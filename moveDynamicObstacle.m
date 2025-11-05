function [hObs, currentPos] = moveDynamicObstacle(hObs, startPt, endPt, size, speed, frame)

    % Interpolate obstacle position
    totalSteps = ceil(norm(endPt - startPt) / speed);
    t = min(frame / totalSteps, 1);
    currentPos = round((1 - t) * startPt + t * endPt);

    % Define square corners
    halfSide = round(size / 2);
    cx = [currentPos(1)-halfSide, currentPos(1)+halfSide, currentPos(1)+halfSide, currentPos(1)-halfSide];
    cy = [currentPos(2)-halfSide, currentPos(2)-halfSide, currentPos(2)+halfSide, currentPos(2)+halfSide];

    % Create or update patch
    if isempty(hObs) || ~isvalid(hObs)
        hObs = patch(cx, cy, 'k', 'FaceAlpha', 1.0, 'EdgeColor', 'none');
    else
        set(hObs, 'XData', cx, 'YData', cy);
    end
end
