clear
clc

%% Load nc
filename='maceio.nc';

if isfile(filename)
    time=ncread(filename,'time');
    lon=double(ncread(filename,'longitude'));
    lat=double(ncread(filename,'latitude'));

    pc=ncread(filename,'particle_count'); %'1-number of particles in a given timestep'
    status= ncread(filename,'status_codes');
    %'0: not_released, 2: in_water, 3: on_land, 7: off_maps, 10: evaporated, 12: to_be_removed, 32: on_tideflat,'

    np=pc(end);
    l = length(lon);
    lon = lon(l-np+1:l);
    lat = lat(l-np+1:l);
    status =status(l-np+1:l);
end

%% step policy


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
t = datetime(2020, 9, 15, 12, 0, 0); % can do t + minutes(5)
[t_year, t_month, t_day, t_hour, t_minute, ~] = datevec(t);
command = strjoin({python_cmd, python_file, num2str(t_year), num2str(t_month), num2str(t_day), num2str(t_hour), num2str(t_minute)}, ' ');
system(command)
