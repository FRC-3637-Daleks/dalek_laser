# OLARTA
## The One localization Algorithem to Rule them All

## About Node
This is a custom node used to localize a position form /map. <br>
This is not exactly production level, many things are hard coded and made specifically for Team3637 2025 Robot.

<br>
This node specifically reads data from /map (witch is extreamly close to being in the odom frame). It also reads lazer and transforms it to the odom frame. Both are converted to pointclouds and compaired together with ICP, the transform is then published to odom_corrected. 
<br>
odom_corrected can then be applied to find the robot position in combination with wheel odom to compensate for error in dead reckoning.

## Specifics
```
/map->map->/PCMap
/scan_filtered->
    scanToPC2-> (makes pointcloud2)
    filterPC2-> (voxel filter)
    ICPCalc-> (To pointcloud, 
                transform to odom,
                statistical outlier filter,
                beef up area around reef,
                ICP (compairs scan to map in odom frame),
                (publish for desmos if enabled),
                Calc transform,
                publish transform to odom_corrected)
```

## Problems to fix
1. Speed (between cycles)
2. Random transforms to insainly wrong spots
3. Starting pos
4. April Tag input