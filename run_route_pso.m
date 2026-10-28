% Load the supplied map and run the original map-based route PSO script.
agvProjectRoot = fileparts(mfilename('fullpath'));
agvMapFile = fullfile(agvProjectRoot, 'data', 'Full_perfect_OCC_grid_workspace.mat');
agvMapData = load(agvMapFile, 'map');
assert(isfield(agvMapData, 'map'), 'The supplied workspace must contain a variable named map.');
map = agvMapData.map;
run(fullfile(agvProjectRoot, 'src', 'routing', 'Swarm_Optimisation_8loc_pick_drop.m'));
