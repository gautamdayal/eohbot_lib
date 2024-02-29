# eoh robot lib

Modules that will be used in our mobile robot autonomy stack. 

### Perception

Simple vision-based perception that uses fiducial tags for localization and object detection. Input: camera feed. Output: Pose estimate + Occupancy 
grid. 

### Planning 

Grid search from current position to goal. Input: occupancy grid. Output: series of coordinates to get to goal. 

### Control

Ensure path computed by planning is following by robot. Input: path plan. Output: the robot works :fire:
