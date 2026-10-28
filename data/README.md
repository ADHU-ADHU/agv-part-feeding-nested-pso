# Map and source provenance

`Full_perfect_OCC_grid_workspace.mat` is the original saved project workspace. It contains a `robotics.OccupancyGrid` object named `map`, together with SLAM scans, poses, and other working variables. The map resolution is 10 cells per meter.

The map-routing entry script loads only `map` rather than restoring the entire workspace:

```matlab
loaded = load('Full_perfect_OCC_grid_workspace.mat', 'map');
map = loaded.map;
```

Use a MATLAB release with compatible robotics/navigation objects. The snapshot is a supplied data artifact; no standalone SLAM map-building script is included in the selected source.

`source-manifest.json` records the relative source location and SHA-256 checksum of each original algorithm, data, and simulation file copied into this repository. The source folder contained duplicate route scripts and generated Simulink caches; the selected files are included once.
