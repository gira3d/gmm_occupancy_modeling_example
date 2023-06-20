CPWD = pwd;
SANDBOX_NAME = 'gira3d-occupancy-modeling';
matches = split(CPWD, SANDBOX_NAME);
SANDBOX = [matches{1}, SANDBOX_NAME];
DATASET = 'mine_001_part3';
DATA_DIR = [SANDBOX, '/data/', DATASET, '/'];
PATH_TO_CONFIG = [SANDBOX, '/wet/src/gmm_occupancy_modeling_example/config/grid3d.yaml'];
GMM_DIR = [DATA_DIR, '100_components/'];

START_SCAN = 1;
LAST_SCAN = 1055;

OCCUPIED_THRESH = 0.7;
FREE_THRESH = 0.13;

PLOT_OCCUPIED_POINTS = 1;
PLOT_FREE_POINTS = 0;

load([DATA_DIR, 'odometry.mat']);

resolution = 0.2; % defined by user
max_range = 15;

all_pclds = [];
grid = Grid3D(PATH_TO_CONFIG);
for i=START_SCAN:5:LAST_SCAN
  time = pointclouds.times(i);
  [~, idx] = odometry.closestTime(time);

  q = odometry.orientations(:, idx);
  R = QuatToR(q);
  t = odometry.positions(:, idx);

  gmm = GMM3();
  gmm.load([GMM_DIR, num2str(i), '.gmm']);
  pcld = gmm.sample(1e5);

  tpcld = transpose(R*transpose(pcld) + repmat(t, 1, size(pcld,1)));

  for j = 1:size(tpcld,1)
    st = t;
    en = transpose(tpcld(j,:));
    trimmed_max_range = max_range - resolution;
    grid.addRay(st, en, trimmed_max_range);
  end
end

% Plot the output
[xyz, probabilities] = grid.getXYZProbability();

if (PLOT_OCCUPIED_POINTS)
  occupied_indices = find(probabilities >= OCCUPIED_THRESH);
  occupied_pts = transpose(xyz(:, occupied_indices));
  pcshow(occupied_pts);
  colormap('jet');
end

if (PLOT_FREE_POINTS)
  free_indices = find(probabilities <= FREE_THRESH);
  free_pts = transpose(xyz(:, free_indices));
  pcshow(pointCloud(free_pts, 'Color', repmat([0, 0, 1], size(free_pts,1), 1)));
end
