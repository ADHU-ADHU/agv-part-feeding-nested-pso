% Run the original nested PSO script and keep its outputs in this workspace.
agvProjectRoot = fileparts(mfilename('fullpath'));
agvNestedPsoDirectory = fullfile(agvProjectRoot, 'src', 'nested_pso');
addpath(agvNestedPsoDirectory);
run(fullfile(agvNestedPsoDirectory, 'with_tour_optimization_diffstyle2.m'));
