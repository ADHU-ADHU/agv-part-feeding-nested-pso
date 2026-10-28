# Setup and execution

## Main nested optimizer

The allocation script and its inner route-ordering function use ordinary MATLAB arrays, random numbers, sorting, and plotting. This part requires **base MATLAB**. A Global Optimization Toolbox implementation of `particleswarm` is not used; both PSO loops are implemented directly in the project source.

Open MATLAB in the repository root and run:

```matlab
run_nested_pso
```

The entry script adds `src/nested_pso` to the path and runs the original allocation script. It leaves results in the calling workspace.

| Variable | Meaning |
| --- | --- |
| `tg_best(1:75)` | Flattened selected allocation tensor |
| `tg_best(end-1)` | Stored best trip count |
| `tg_best(end)` | Stored cost/penalty score |
| `best_value_tour` | Trip-count convergence history |

To inspect and validate a resulting allocation manually:

```matlab
allocation = reshape(tg_best(1:75), 3, 5, 5);
delivered = sum(allocation, 3);
tripLoads = squeeze(sum(sum(allocation, 1), 2));

assert(all(allocation(:) >= 0) && all(allocation(:) == round(allocation(:))));
assert(isequal(delivered, Demand_matrix), 'Per-material station demand is not satisfied');
assert(all(tripLoads <= robot_limit), 'A trip exceeds vehicle capacity');
```

These checks are important because the original repair and scoring code can accept an incorrect distribution even when its aggregate delivered count matches total demand. They may therefore flag an output of the original prototype; they are not a claim that every stored best allocation is feasible.

The supplied parameters are 100 outer particles, 150 outer iterations, 100 inner particles, and 10 inner iterations. The inner optimizer is called repeatedly during outer candidate evaluation, so this is considerably more work than one PSO run.

## Separate map-based routing demo

The source uses `plannerAStarGrid`, occupancy maps, map inflation, and world/grid conversion. These require compatible MathWorks navigation/robotics functionality; see the [A* planner documentation](https://www.mathworks.com/help/nav/ref/plannerastargrid.html).

Run from the repository root:

```matlab
run_route_pso
```

This loads only the saved `map` object and runs the original PSO route script. It produces map/route plots, convergence data, and `Final_path` as world-coordinate waypoints. The default PSO route problem is separate from the outer allocator's problem data.

The GA and DE files can be run with `map` already loaded:

```matlab
run('src/routing/Genetic_algorithm_8loc_pic_drop.m')
% Or, in a separate run:
run('src/routing/Differential_evolution_8loc_.m')
```

Match demand, return-to-start behavior, stopping budgets, and random seeds before drawing algorithm comparisons. In the supplied versions, GA/DE use the full demand matrix and include a return leg; PSO uses a subset and omits that leg.

## Optional Simulink simulation

The supplied model metadata identifies **R2022a Update 1**. Treat that as the reference environment for this legacy model, not a promise of compatibility with every MATLAB release.

The model references Pure Pursuit, virtual-world/VR blocks, Stateflow, vision, and image-processing blocks. Its installed toolbox/license requirements must be checked in MATLAB. The [Pure Pursuit documentation](https://www.mathworks.com/help/nav/ref/purepursuit.html) describes its waypoint-following role.

From a session in which a route has been generated:

```matlab
projectRoot = pwd;
cd(fullfile(projectRoot, 'simulation'))
open_system('AGV_Simulation_Follow.slx')
```

Inspect the model's required base-workspace variables, including `Final_path`, `lastx`, `lasty`, and `start`, before running it. The routing scripts reuse `start` for individual A* legs, so its final value is not necessarily the intended initial robot pose.

The simulation uses `Scenario.x3d` and the supplied brick texture. Some model/world references still point to an unavailable `Scenario.WRL` or an absolute Windows wheel-texture path. These need resolution in the original simulation environment before treating it as a turnkey demo.

`Maze_Creator.m` can regenerate the X3D environment using `ScenarioTemplate.x3d`. It also creates a numeric variable named `map`; run it separately from the routing workspace to avoid replacing the occupancy-map object.

The environment builder uses legacy `vrworld` APIs. MathWorks documents their [removal in R2026b](https://www.mathworks.com/help/sl3d/vrworld.html), so a current installation may require migration of this optional visualization. The base-MATLAB nested optimizer is independent of those VR APIs.

## Verification scope

Repository preparation checks source checksums, selected assets, internal links, and the archive structure/metadata of the Simulink model. MATLAB and Simulink were not available for execution. Reported historical examples are not presented as freshly reproduced benchmark results.
