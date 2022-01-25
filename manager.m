clear
clc

% Time
t = datetime(2020, 9, 15, 12, 0, 0);
tf = datetime(2020, 9, 20, 12, 0, 0);
t_step = minutes(10);

% Load shp file for Alagoas
sl_alagoas = shaperead('.\shp\BRA_admin_AL.shp');

% Getting region of interest
region = kml2struct('search_region.kml');
res_grid = 111/0.5;
width = ceil(res_grid * (region.BoundingBox(2,1) - region.BoundingBox(1,1)));
height = ceil(res_grid * (region.BoundingBox(2,2) - region.BoundingBox(1,2)));
grid = zeros(height, width);
mask = zeros(height, width);
dist_grid = zeros(height, width);
for i = 1:width
    for j = 1:height
        if inpolygon((i/res_grid) + region.BoundingBox(1,1), (j/res_grid) + region.BoundingBox(1,2), region.Lon, region.Lat) == 0
            grid(j, i) = -Inf;
            mask(j, i) = 1;
        else
            dist_grid(j, i) = res_grid * min(sqrt(((i/res_grid) + region.BoundingBox(1,1) - sl_alagoas(1).X).^2 + ((j/res_grid) + region.BoundingBox(1,2) - sl_alagoas(1).Y).^2));
        end
    end
end
max_dist=max(max(dist_grid));
dist_grid = 1/max_dist*5*(~mask.*max_dist-dist_grid)+grid;

x = linspace(region.BoundingBox(1,1), region.BoundingBox(2,1), width);
y = linspace(region.BoundingBox(1,2), region.BoundingBox(2,2), height);
[X, Y] = meshgrid(x, y);

grid_initial = grid;

lon = 0;
lat = 0;

prev_simul = true;
release = 1;
%%
% 1 day loop
if (prev_simul)
    while t < tf
        t
        if (lon == 0)
            [lon, lat] = gnome_sim(t, release);
        else
            [lon, lat] = gnome_sim(t, release, lon, lat);
        end
        
        % Define limits
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
        
        % Histogram data
        % figure
        % h = histogram2(lonI,latI,size(grid),'FaceColor','flat');
        % colormap jet
        % view(2)
        %grid,
        % axis equal, ylabel('Latitude');xlabel('Longitude');
        % hcb = colorbar; set(hcb,'fontname','calibri','fontsize',12);
        % hcb.Label.String = 'No of Particles';
        %mapshow(sl_brasil,'FaceColor',[1 1 1],'HandleVisibility','off');
        %grid,
        % axis equal, axis([xmin xmax ymin ymax]);
        % set(gca,'YTick',(ymin:1:ymax));
        % title('Particle trajectories','fontsize',12);
        % set(gca,'fontname','calibri','fontsize',fntsz);
        [h, ~, ~] = histcounts2(latI,lonI,size(grid));
        
        % Prepare grid with histogram values and boundaries
        for i = 1:size(grid, 1)
            for j = 1:size(grid, 2)
                if (grid(i, j) > -1)
                    grid(i, j) = h(i, j);
                end
            end
        end
        
        t = t + t_step;
        %lon = lonI;
        %lat = latI;
    end
end
%%
t = tf;
tf = datetime(2020, 9, 16, 15, 0, 0);

[lon, lat] = gnome_sim(t,release);
% Define limits
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

% Prepare grid with histogram values and boundaries
for i = 1:size(grid, 1)
    for j = 1:size(grid, 2)
        if (grid(i, j) > -1)
            grid(i, j) = h(i, j);
        end
    end
end



n_robots = 7;
heading = zeros(n_robots, 1);
robots = [1, 15; 1, 16; 1, 17;2 15;2 16;2 17;3 15];
% weights = [omega_concentration omega_sensitivity omega_distance
% omega_neighbors]
c = ['r'; 'g';'y'; 'c'; 'm';'w'; 'k'];
weights =...
    [2.0 0.1 0.3 0.2 1;
    2.0 0.1 0.3 0.2 1;
    2.0 0.1 0.3 0.2 1;
    0.1 2.0 0.5 0.1 1;
    0.1 2.0 0.5 0.1 1;
    0.1 2.0 0.5 0.1 1;
    0.1 2.0 0.5 0.1 1];
path_robots = zeros(n_robots, (tf - t)/minutes(1) + 1, 2);
for robot = 1:n_robots
    path_robots(robot, 1, 1) = robots(robot, 1);
    path_robots(robot, 1, 2) = robots(robot, 2);
end
cnt = 2;

release_cnt = 0;

[row, col] = find(~mask');

xls = mean([xEdges(1:end-1);xEdges(2:end)]);
yls = mean([yEdges(1:end-1);yEdges(2:end)]);
[xx yy] = meshgrid(xls,yls);

positions = [xx(:) yy(:)];

while (t < tf)
    
    for it = 1:3
        
        % Define limits
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
        
        [h, ~, ~, binY, binX] = histcounts2(latI,lonI,size(grid));
        
        lonp=[];
        latp=[];
        for k=1:length(row)
            lonp = [lonp;lon(I(binX==row(k) & binY==col(k)))];
            latp = [latp;lat(I(binX==row(k) & binY==col(k)))];
        end
        
        [f,~] = ksdensity([lonp latp],positions,'Bandwidth',0.02);
        f= reshape(f,size(grid));
        
        grid = 5/max(max(f))*~mask.*f.*(h>0)+grid_initial;
        
        [robots, heading] = reactive_patrol(grid, robots, heading, mask, dist_grid,weights);
        
        % Consume particles
        for robot = 1:n_robots
            h(robots(robot, 2), robots(robot, 1)) = 0;
            grid(robots(robot, 2), robots(robot, 1)) = 0;
            % binX and binY address the indexes from histcounts2, and I has
            % the indexes on whole coastal range lat lon.
            lon(I(binX==robots(robot, 1) & binY==robots(robot, 2))) = NaN;
            lat(I(binX==robots(robot, 1) & binY==robots(robot, 2))) = NaN;
            
            % Saving path
            path_robots(robot, cnt, 1) = robots(robot, 1);
            path_robots(robot, cnt, 2) = robots(robot, 2);
        end
        
        % Removing NaN particles
        lon = lon(~isnan(lon));
        lat = lat(~isnan(lat));
        
        t = t + seconds(40);
        
        figure(2)
        
        pcolor(X, Y, grid);
        set(gca, 'YDir', 'normal');
        hold on
        mapshow(sl_alagoas,'FaceColor',[1 1 1],'HandleVisibility','off');
        title(string(t));ylabel('Latitude');xlabel('Longitude'); axis equal, axis([xmin xmax ymin ymax]);
        caxis([-1, 5])
        colormap jet
        colorbar
        for robot = 1:n_robots
            scatter(region.BoundingBox(1,1) + (robots(robot, 1)-0.5)/res_grid, ...
                region.BoundingBox(1,2) + (robots(robot, 2)-0.5)/res_grid, ...
                50, c(robot, :), 'filled');
        end
        drawnow
        tstr=t;
        tstr.Format='ddMMuuuu-HH-mm-ss';
        saveas(gcf,'pos'+string(tstr)+'.png')
        for robot = 1:n_robots
            pp=plot((path_robots(robot, 1:cnt, 1) - 0.5)/res_grid + region.BoundingBox(1,1), (path_robots(robot, 1:cnt, 2) - 0.5)/res_grid + region.BoundingBox(1,2), c(robot, :), 'LineWidth', 5);
            pp.Color(4) = 0.4;
            scatter(region.BoundingBox(1,1) + (robots(robot, 1)-0.5)/res_grid, ...
                region.BoundingBox(1,2) + (robots(robot, 2)-0.5)/res_grid, ...
                50, c(robot, :), 'filled');
        end
        
        hold off
        drawnow
        saveas(gcf,'traj'+string(tstr)+'.png')
        %pause
        
        cnt = cnt + 1;
    end
    
    release_cnt = release_cnt + 1;
    if (release_cnt == 5)
        % Passed 10 minutes
        release = 1;
        release_cnt = 0;
    else
        % Do not release new particles
        release = 0;
    end
    
    %     % Removing NaN particles
    %     lon = lon(~isnan(lon));
    %     lat = lat(~isnan(lat));
    
    [lon, lat] = gnome_sim(t,release, lon, lat);
    
end

pcolor(X, Y, grid);
set(gca, 'YDir', 'normal');
hold on
mapshow(sl_alagoas,'FaceColor',[1 1 1],'HandleVisibility','off');
ylabel('Latitude');xlabel('Longitude'); axis equal, axis([xmin xmax ymin ymax]);
for robot = 1:n_robots
    scatter(region.BoundingBox(1,1) + (robots(robot, 1)-0.5)/res_grid, ...
        region.BoundingBox(1,2) + (robots(robot, 2)-0.5)/res_grid, ...
        50, c(robot, :), 'filled'); % Need that 0.5 because pcolor is based on vertices
    plot((path_robots(robot, :, 1) - 0.5)/res_grid + region.BoundingBox(1,1), (path_robots(robot, :, 2) - 0.5)/res_grid + region.BoundingBox(1,2), c(robot, :), 'LineWidth', 5);
end
hold off
caxis([-1, 5])
title('omega_0 = -0.02; omega_1 = 0.07');
colormap jet
colorbar