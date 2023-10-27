function range = exampleHelperAvoidObstaclesRangeSensor(pose, mapMatrix, maxRange, mapScale)
% This function may be removed in the future.
% Copyright The MathWorks Inc, 2023
persistent r
 if isempty(r)
    r = rangeSensor('HorizontalAngle', [-3*pi/8, 3*pi/8], 'HorizontalAngleResolution', pi/8, 'Range', [0 maxRange]);
end
map = binaryOccupancyMap(mapMatrix, mapScale);
p = pose';

% The range sensor accepts a pose on the map which is within the map's
% limits. Thus, filter out positions outside the map, and override them
% with readings that are zero.
xcord=p(:,1);
ycord=p(:,2);
xmin=map.XWorldLimits(1);
ymin=map.YWorldLimits(1);
xmax=map.XWorldLimits(2);
ymax=map.YWorldLimits(2);
isxvalid=xcord>=xmin&xcord<=xmax;
isyvalid=ycord>=ymin&ycord<=ymax;
isInside=isxvalid&isyvalid;
range = zeros(r.NumReadings,size(p,1));
range(:,isInside) = r(p(isInside,:), map);
end
