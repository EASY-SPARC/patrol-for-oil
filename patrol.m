clear
clc

%% Simulation Parameters
% Time
t = datetime(2020, 9, 15, 12, 0, 0);

%% Getting region of interest
region = kml2struct('search_region.kml');

res_grid = 111/0.5;
width = ceil(res_grid * (region.BoundingBox(2,1) - region.BoundingBox(1,1)));
height = ceil(res_grid * (region.BoundingBox(2,2) - region.BoundingBox(1,2)));
grid = zeros(height, width);
for i = 1:width
    for j = 1:height
        if inpolygon((i/res_grid) + region.BoundingBox(1,1), (j/res_grid) + region.BoundingBox(1,2), region.Lon, region.Lat) == 0
            grid(j, i) = -Inf;
        end
    end
end
x = linspace(region.BoundingBox(1,1), region.BoundingBox(2,1), width);
y = linspace(region.BoundingBox(1,2), region.BoundingBox(2,2), height);
figure()
imagesc(x, y, grid);
set(gca,'YDir','normal'); % imagesc flips y axis by default, this line reverts that
caxis([-1, 3]);
colormap(jet);
colorbar

%% Load nc
filename='maceio.nc';

time=ncread(filename,'time');
lon=double(ncread(filename,'longitude'));
lat=double(ncread(filename,'latitude'));

pc=ncread(filename,'particle_count'); %'1-number of particles in a given timestep'
status = ncread(filename,'status_codes');
%'0: not_released, 2: in_water, 3: on_land, 7: off_maps, 10: evaporated, 12: to_be_removed, 32: on_tideflat,'

np=pc(end);
l = length(lon);
lon = lon(l-np+1:l);
lat = lat(l-np+1:l);
status =status(l-np+1:l);

% Find particles "in water - status=2"
I=find(status==2);
lonI=lon(I,:);
latI=lat(I,:);

% Define limits
xmin=-36.6;
xmax=-34.5;
ymin=-10.8;
ymax=-8.7;

fntsz = 16;

I=find(lonI<=xmax);
lonI=lonI(I,:);
latI=latI(I,:);
I=find(lonI>=xmin);
lonI=lonI(I,:);
latI=latI(I,:);
I=find(latI>=ymin);
lonI=lonI(I,:);
latI=latI(I,:);
I=find(latI<=ymax);
lonI=lonI(I,:);
latI=latI(I,:);

% Running KDE for concentration
data = [lonI latI];
[bw, de, xk, yk] = kde2d(data);

%% Prepare grid
for j = 1:size(grid, 1)
    for i = 1:size(grid, 2)
        if (grid(j, i) > -1)
            lon = (i/res_grid) + region.BoundingBox(1,1);
            lat = (j/res_grid) + region.BoundingBox(1,2);
            
            grid(j, i) = interpolate(lon, lat, de, xk(1, :), yk(:, 1));
        end
    end
end
imagesc(x, y, grid);
set(gca,'YDir','normal');
colormap(jet);
colorbar

%% step policy
grid = 10^8 * grid; % Scaling

%% Comsume particles where there are robots


%% Write txt
output_filename = 'step.txt';

if isfile(output_filename)
    delete(output_filename);
end

output_file = fopen(output_filename, 'w');
fprintf(output_file, '%.10f\t%.10f\t1\n', [lat'; lon']);
fclose(output_file);

%% Call step.py using new time ref
%python_cmd = 'C:\Users\glaub\.conda\envs\gnome\python.exe'; % Windows
python_cmd = '/home/glauberrleite/miniconda3/envs/gnome/bin/python'; % Linux
python_file = 'step.py';
t = t + minutes(5); % 5 minutes step
[t_year, t_month, t_day, t_hour, t_minute, ~] = datevec(t);
command = strjoin({python_cmd, python_file, num2str(t_year), num2str(t_month), num2str(t_day), num2str(t_hour), num2str(t_minute)}, ' ');
system(command)

%% Functions
function target = computeTargetMulti(pos, heading, omega_0, omega_1, grid, neigh)
    max_value = 0;
    target = pos;
    max_heading = heading + pi;
    for i = 1:size(grid, 2)
        for j = 1:size(grid, 1)
            if grid(j, i) < 0 % out of border conditions
                continue;
            else
                current = [i, j];
                if any(current ~= pos)
                    move = current - pos;
                    distance = norm(move);
                    new_heading = atan2(move(2), move(1));

                    distance_nearest_neigh = Inf;
                    for k = 1:size(neigh, 1)
                        distance_neigh = norm(current - neigh(k, :));
                        if (distance_neigh < distance_nearest_neigh)
                            distance_nearest_neigh = distance_neigh;
                        end
                    end

                    value = max(grid(j, i) + omega_0 * distance + omega_1 * distance_nearest_neigh, 0);
                    if (value > max_value) || (value == max_value && abs(new_heading - heading) < abs(max_heading - heading))
                        target = current;
                        max_value = value;
                        max_heading = new_heading;
                    end
                end
            end
        end
    end
    if (max_value == 0)
        target = pos;
    end
end

function value = interpolate(current_lon, current_lat, values, ref_lon, ref_lat)
    delta_lon = ref_lon - current_lon;
    delta_lat = ref_lat - current_lat;
    
    upper_lon = min(delta_lon(delta_lon > 0)) + current_lon;
    lower_lon = max(delta_lon(delta_lon < 0)) + current_lon;
    upper_lat = min(delta_lat(delta_lat > 0)) + current_lat;
    lower_lat = max(delta_lat(delta_lat < 0)) + current_lat;
    
    south_west = values(ref_lon == lower_lon, ref_lat == lower_lat);
    south_east = values(ref_lon == upper_lon, ref_lat == lower_lat);
    north_west = values(ref_lon == lower_lon, ref_lat == upper_lat);
    north_east = values(ref_lon == upper_lon, ref_lat == upper_lat);
    
    A = [upper_lon - current_lon, current_lon - lower_lon];
    B = [upper_lat - current_lat, current_lat - lower_lat]';
    Q = [south_west, north_west; south_east, north_east];
    
    value = ((upper_lon - lower_lon) * (upper_lat - lower_lat))^1 * A * Q * B;
end