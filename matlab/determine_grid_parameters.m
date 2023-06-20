minx = -105;
maxx = 15;

miny = -95;
maxy = 15;

minz = -10;
maxz = 20;

% Adjust the resolution to determine parameters for the occupancy grid
% map resolution
resolution = 0.1;

disp('Computing width, height, depth values for example...');
disp('Please replace the following parameters in the yaml file under map_utils_matlab/config/grid3d.yaml');
disp(['width:       ', num2str(uint32( (maxx - minx) / resolution))]);
disp(['height:      ', num2str(uint32( (maxy - miny) / resolution))]);
disp(['depth:       ', num2str(uint32( (maxz - minz) / resolution))]);
disp(['resolution:  ', num2str(resolution)]);
