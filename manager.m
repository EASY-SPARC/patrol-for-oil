clear
clc

% Time
t = datetime(2020, 9, 15, 12, 0, 0);
tf = datetime(2020, 9, 16, 12, 0, 0);
t_step = minutes(10);

% Getting region of interest
region = kml2struct('search_region.kml');
res_grid = 111;
width = ceil(res_grid * (region.BoundingBox(2,1) - region.BoundingBox(1,1)));
height = ceil(res_grid * (region.BoundingBox(2,2) - region.BoundingBox(1,2)));
grid = zeros(height, width);
mask = zeros(height, width);
for i = 1:width
    for j = 1:height
        if inpolygon((i/res_grid) + region.BoundingBox(1,1), (j/res_grid) + region.BoundingBox(1,2), region.Lon, region.Lat) == 0
            grid(j, i) = -Inf;
            mask(j, i) = 1;
        end
    end
end
x = linspace(region.BoundingBox(1,1), region.BoundingBox(2,1), width);
y = linspace(region.BoundingBox(1,2), region.BoundingBox(2,2), height);
[X, Y] = meshgrid(x, y);

% Load shp file for Alagoas
sl_alagoas = shaperead('.\shp\BRA_admin_AL.shp');

grid_initial = grid;

lon = 0;
lat = 0;

prev_simul = false;
release = 1;

% 1 day loop
if (prev_simul)
    while t < tf

        if (lon == 0)
           [lon, lat] = gnome_sim(t);
        else
           [lon, lat] = gnome_sim(t, lon, lat, release);
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
        [h, ~, ~] = histcounts2(lonI,latI,size(grid));

        % Prepare grid with histogram values and boundaries
        for j = 1:size(grid, 1)
            for i = 1:size(grid, 2)
                if (grid(j, i) > -1)
                    grid(j, i) = h(i, j);
                end
            end
        end

        t = t + t_step;
        %lon = lonI;
        %lat = latI;
    end
else
    [lon, lat] = gnome_sim(t);
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
    
    [h, ~, ~, binX, binY] = histcounts2(lonI,latI,size(grid));

    % Prepare grid with histogram values and boundaries
    for j = 1:size(grid, 1)
        for i = 1:size(grid, 2)
            if (grid(j, i) > -1)
                grid(j, i) = h(i, j);
            end
        end
    end

end

n_robots = 3;
heading = zeros(n_robots, 1);
robots = [1, 15; 1, 16; 1, 17];
path_robots = zeros(n_robots, (tf - t)/minutes(1) + 1, 2);
for robot = 1:n_robots
    path_robots(robot, 1, 1) = robots(robot, 1);
    path_robots(robot, 1, 2) = robots(robot, 2);
end
cnt = 2;

release_cnt = 0;

while (t < tf)
    for it = 1:3
        [robots, heading] = reactive_patrol(grid, robots, heading, mask);
               
        % Consume particles
        for robot = 1:n_robots
            grid(robots(robot, 2), robots(robot, 1)) = 0;
            
            % binX and binY address the indexes from histcounts2, and I has
            % the indexes on whole coastal range lat lon.
            lon(I(binX==robots(robot, 1) & binY==robots(robot, 2))) = NaN;
            lat(I(binX==robots(robot, 1) & binY==robots(robot, 2))) = NaN;
            
            % Saving path
            path_robots(robot, cnt, 1) = robots(robot, 1);
            path_robots(robot, cnt, 2) = robots(robot, 2);
        end
        
        t = t + seconds(40);
        
        figure(2)
        c = ['m'; 'w'; 'k'];
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
    
    % Removing NaN particles
    lon = lon(~isnan(lon));
    lat = lat(~isnan(lat));
    
    [lon, lat] = gnome_sim(t, lon, lat, release);
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
    
    [h, ~, ~, binX, binY] = histcounts2(lonI,latI,size(grid));
    grid = grid_initial;
    % Prepare grid with histogram values and boundaries
    for j = 1:size(grid, 1)
        for i = 1:size(grid, 2)
            if (grid(j, i) > -1)
                grid(j, i) = h(i, j);
            end
        end
    end
    
end

c = ['m'; 'w'; 'k'];
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