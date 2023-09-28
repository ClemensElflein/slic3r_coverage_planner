# slic3r_coverage_planner
A coverage planner for ROS using libslic3r as core logic

# Parameter
## Clockwise
By default, outer perimeter gets followed counter clockwise. 
With the optional parameter clockwise, (true/false) you can change the direction.

```
    <node pkg="slic3r_coverage_planner" type="slic3r_coverage_planner" name="slic3r_coverage_planner" output="screen">
      <param name="clockwise" type="bool" value="true" />
    </node>
```

## equally_spaced
By default, coverage planner calculates a path which consists of a pose every 10 cm, even on straight lines. For example a path from point (0,0) to (1,0) will result in a path with 10 poses.
This can cause issues when using other planners than ftc_local_planner. By setting the optional parameter equally_spaced (true/false), you can change this behavior. If set to false, the path will only contain poses where needed (i.e. direction changes). 

```
    <node pkg="slic3r_coverage_planner" type="slic3r_coverage_planner" name="slic3r_coverage_planner" output="screen">
      <param name="equally_spaced" type="bool" value="false" />
    </node>
```
