clear
clc
sf = 0.05;
% Getting region
region = kml2struct('search_region.kml');

res_grid = 111/0.5;
width = ceil(res_grid * (region.BoundingBox(2,1) - region.BoundingBox(1,1)));
height = ceil(res_grid * (region.BoundingBox(2,2) - region.BoundingBox(1,2)));
grid = zeros(height, width);
x = linspace(region.BoundingBox(1,1), region.BoundingBox(2,1), width);
y = linspace(region.BoundingBox(1,2), region.BoundingBox(2,2), height);

sl_alagoas = shaperead('.\shp\BRA_admin_AL.shp');

mask = zeros(height, width);
dist_grid = zeros(height, width);

for i = 1:width
    for j = 1:height
        if inpolygon((i/res_grid) + region.BoundingBox(1,1), (j/res_grid) + region.BoundingBox(1,2), region.Lon, region.Lat) ~= 0
            dist_grid(j, i) = res_grid * min(sqrt(((i/res_grid) + region.BoundingBox(1,1) - sl_alagoas(1).X).^2 + ((j/res_grid) + region.BoundingBox(1,2) - sl_alagoas(1).Y).^2));
        end
    end
end
max_dist=max(max(dist_grid));
dist_grid = 1/max_dist*5*(~mask.*max_dist-dist_grid)+grid;

grid_initial = grid;

filename = 'maragogi.nc';

lon = double(ncread(filename,'longitude'));
lat = double(ncread(filename,'latitude'));

pc = ncread(filename,'particle_count'); %'1-number of particles in a given timestep'
status = ncread(filename,'status_codes');

np = pc(end);
l = length(lon);
lon = lon(l-np+1:l);
lat = lat(l-np+1:l);
status = status(l-np+1:l);

% Find particles "in water - status=2"
I=find(status==2);
lon=lon(I,:);
lat=lat(I,:);

xmin = region.BoundingBox(1,1);
xmax = region.BoundingBox(2,1);
ymin = region.BoundingBox(1,2);
ymax = region.BoundingBox(2,2);

I1=find(lon<=xmax);
lonI=lon(I1,:);
latI=lat(I1,:);
I2=find(lonI>=xmin);
lonI=lonI(I2,:);
latI=latI(I2,:);
I3=find(latI>=ymin);
lonI=lonI(I3,:);
latI=latI(I3,:);
I4=find(latI<=ymax);
lonI=lonI(I4,:);
latI=latI(I4,:);

I = I1(I2(I3(I4)));

[h, yEdges, xEdges, binY, binX] = histcounts2(latI,lonI,size(grid));

[row, col] = find(~mask');
xls = mean([xEdges(1:end-1);xEdges(2:end)]);
yls = mean([yEdges(1:end-1);yEdges(2:end)]);
[xx, yy] = meshgrid(xls,yls);

positions = [xx(:) yy(:)];

lonp=[];
latp=[];
for k=1:length(row)
    lonp = [lonp;lon(I(binX==row(k) & binY==col(k)))];
    latp = [latp;lat(I(binX==row(k) & binY==col(k)))];
end
%%
[f,~] = ksdensity([lonp latp],positions,'Bandwidth',0.005);
f = reshape(f,size(grid));

grid = 5/max(max(f))*~mask.*f.*(h>0)+grid_initial;

f_new = 3 * f/max(max(f));

figure()
imagesc(x, y, h);
set(gca,'YDir','normal'); % imagesc flips y axis by default, this line reverts that
hold on
mapshow(sl_alagoas,'FaceColor',[1 1 1],'HandleVisibility','off');
hold off
caxis([0, 3]);
colormap(jet);
colorbar

figure()
imagesc(x, y, f_new);
set(gca,'YDir','normal'); % imagesc flips y axis by default, this line reverts that
hold on
mapshow(sl_alagoas,'FaceColor',[1 1 1],'HandleVisibility','off');
hold off
caxis([0, 3]);
colormap(jet);
colorbar

figure()
imagesc(x, y, grid);
set(gca,'YDir','normal'); % imagesc flips y axis by default, this line reverts that
hold on
mapshow(sl_alagoas,'FaceColor',[1 1 1],'HandleVisibility','off');
hold off
caxis([0, 3]);
colormap(jet);
colorbar