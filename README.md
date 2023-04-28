# ROS spatial utilities

**NOTE**: I no longer work in the field of robotics, this is not maintained. But
hopefully the ideas and techniques presented here might be of use to you. Or the
software itself might even still work!

---

This is a utility library with various useful functions related to spatial
data structures and calculations.

In this library currently:
* 2D pose class optimised for needs in Monte Carlo Localisation (`pose_2d.h`).
* Map<->world coordinate conversion & in bounds logic (`scaled_map.h`). Both
  infinite and bounded maps are supported. Bounded maps can have an offset and
  maps are assumed to be axis aligned.
* Map class (`map.h`, `map_state.h`, `map_types.h`). This supports loading data
  from `nav_msgs/OccupancyGrid`. There is functionality to generate random poses
  in free space as well as querying map state.
* Angle utilities (angle delta, normalisation, conversion) (`angle.h`)
* Distance-from-obstacles map generation (`distance_calculator.h`, `map_types.h`).

This is based on code extracted from [QuickMCL](https://github.com/VorpalBlade/quickmcl).
