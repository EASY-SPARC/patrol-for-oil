clear
clc

% Time
t = datetime(2020, 9, 15, 12, 0, 0);
tf = datetime(2020, 9, 16, 12, 0, 0);
t_step = minutes(15);

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

grid_initial = grid;

prev_simul = false;

lon = 0;
lat = 0;

% 1 day loop
if (prev_simul)
    while t < tf

        if (lon == 0)
           [lon, lat] = gnome_sim(t);
        else
           [lon, lat] = gnome_sim(t, lon, lat);
        end

        % Define limits
        xmin = region.BoundingBox(1,1);
        xmax = region.BoundingBox(2,1);
        ymin = region.BoundingBox(1,2);
        ymax = region.BoundingBox(2,2);

        I=find(lon<=xmax);
        lonI=lon(I,:);
        latI=lat(I,:);
        I=find(lonI>=xmin);
        lonI=lonI(I,:);
        latI=latI(I,:);
        I=find(latI>=ymin);
        lonI=lonI(I,:);
        latI=latI(I,:);
        I=find(latI<=ymax);
        lonI=lonI(I,:);
        latI=latI(I,:);

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
cnt = 15;

while (t < tf)
    for it = 1:cnt
        [robots, heading] = reactive_patrol(grid, robots, heading, mask);
               
        % Consume particles
        for robot = 1:n_robots
            grid(robots(robot, 2), robots(robot, 1)) = 0;
            
            % binX and binY address the indexes from histcounts2, and I has
            % the indexes on whole coastal range lat lon.
            lon(I(binX==robots(robot, 1) & binY==robots(robot, 2))) = [];
            lat(I(binX==robots(robot, 1) & binY==robots(robot, 2))) = [];
        end
        
        t = t + minutes(1);
        
        c = ['green'; 'white'; 'black'];
        imagesc(grid);
        set(gca, 'YDir', 'normal');
        hold on
        for robot = 1:n_robots
            scatter(robots(robot, 1), robots(robot, 2), [], c(robot, :), 'filled');
        end
        hold off
        caxis([-1, 5])
        colorbar
        pause
        
    end
    [lon, lat] = gnome_sim(t, lon, lat);
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
