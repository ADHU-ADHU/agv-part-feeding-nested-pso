# Implementation notes

This repository preserves the selected original optimization scripts. These notes describe their actual behavior and the main considerations when reproducing or extending the project.

## Outer and inner objectives

The outer script assigns three materials across five destinations and up to five trips. It calculates a load-weighted route cost, a trip penalty, and a return component. Its personal/global-best comparisons, however, use only the stored trip count. Equal-trip allocations are not compared by cost.

The inner PSO searches pickup and delivery order for a given trip and minimizes its computed route cost. The two levels therefore do not implement a fully cost-optimal joint solver, despite calculating costs throughout the process.

## Demand repair and route endpoints

The outer repair condition compares allocations against the original demand matrix, then clips against remaining demand only when that condition triggers. A trip can exceed remaining demand without exceeding original demand. Overdelivery at one station/material and underdelivery elsewhere may cancel in the aggregate delivered count.

The inner objective filters inactive pickup and delivery locations when calculating a route's cost. Its final returned permutations are unfiltered. The outer script treats the final returned delivery index as the actual trip endpoint, potentially distorting the cost of moving to the next trip and returning home.

A candidate with no successfully evaluated trip can also reach return-cost indexing with a zero or stale delivery endpoint. These original edge cases are retained in the source.

Validate the full allocation tensor against per-material, per-station demand and trip capacity rather than relying only on the printed best count or scalar score. The setup guide includes explicit MATLAB checks.

## Randomness and computation

The outer script seeds the random generator with `rng(1)`. The inner helper calls `rng(5)` on every invocation, resetting MATLAB's shared random stream. Inner calls therefore also affect the outer optimizer's subsequent random draws.

Both searches use handwritten PSO loops. Their original inertia, population, iteration, and integer-repair settings are preserved; no tuning or new convergence measurements were performed during publication preparation.

## Ordering versus geometric paths

The nested optimizer uses embedded distance tables. It does not produce an obstacle-grid path or automatically feed selected trip allocations into the standalone map-routing script.

The standalone PSO, GA, and DE scripts calculate A* leg paths on a supplied occupancy grid. Their distance sums are in grid-cell units; the saved map has 10 cells per meter. Cost factors are model coefficients, not measured energy use or currency.

The standalone PSO has its own subset demand and comments out its return leg. GA and DE use the full demand and include the return. All three standalone route scripts retain visits to all pickup/delivery indices, including zero-demand locations. Their original outputs should not be compared as equivalent benchmark instances.

## Historical results and supporting assets

The accompanying project report contains qualitative simulations and example cost/allocation tables. Its illustrated convergence and tabulated trip schedule are not fully consistent, and one displayed cost total does not exactly match the sum of rounded table entries. The README therefore focuses on the formulation, source, and visual outputs instead of asserting a reproduced improvement percentage or certified minimum.

The saved map is an existing project snapshot; a complete standalone SLAM reconstruction script is not part of the selected source. The Simulink model and virtual-world assets are provided as simulation context. They contain legacy and missing-file references documented in the setup guide.

The original report, its review annotations, presentation file, duplicate drafts, generated caches, and scratch simulation outputs are not redistributed here. Original copied files are listed with checksums in `data/source-manifest.json`.
