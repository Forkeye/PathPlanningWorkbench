function [mapData, img_display, rows, cols] = preprocessMap(imageFilePath, mode)
    % Read image
    img = imread(imageFilePath);
    
    % Convert to grayscale if RGB
    if size(img, 3) == 3
        img = rgb2gray(img);
    end

    % Optional contrast enhancement and sharpening
    if std(double(img(:))) < 40
        img_eq = adapthisteq(img);
        img_sharp = imsharpen(img_eq, 'Radius', 2, 'Amount', 1.5);
    else
        img_sharp = img;
    end

    if strcmp(mode, 'grid')
        % Binary grid for A*/Dijkstra
        binaryMap = imbinarize(img_sharp, 0.65);
        se = strel('disk', 2);
        binaryMap = imdilate(binaryMap, se);
        mapData = double(binaryMap);
        [rows, cols] = size(mapData);
        img_display = img;

    elseif strcmp(mode, 'occupancy')
        % Occupancy map for RRT/Potential Fields
        imageNorm = mat2gray(img_sharp);
        imageOccupancy = 1 - imageNorm;
        mapData = occupancyMap(imageOccupancy, 1);
        [rows, cols] = size(imageOccupancy);

        % Image for display (flipped for RRT/PF visualization)
        %img_display = flipud(img);
        img_display = img;
    else
        error('Invalid mode. Use ''grid'' or ''occupancy''.');
    end
end
