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