function [minX, maxX, minY, maxY] = scaleMap(s)
m = dlmread(s);
minX = min([m(:,1); m(:, 3)]);
maxX = max([m(:,1); m(:, 3)]);
minY = min([m(:,2); m(:, 4)]);
maxY = max([m(:,2); m(:, 4)]);
dx = 0.5 * (minX + maxX);
dy = 0.5 * (minY + maxY);
m(:,1) = m(:,1) - dx;
m(:,3) = m(:,3) - dx;
m(:,2) = m(:,2) - dy;
m(:,4) = m(:,4) - dy;
dlmwrite(s, m, "precision", "%10f");
end
