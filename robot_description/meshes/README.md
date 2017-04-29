# Meshes

```
https://grabcad.com/library/differential-drive-robot-1
https://github.com/ethz-asl/ros_best_practices/wiki
http://blogs.solidworks.com/teacher/wp-content/uploads/sites/3/WPI-Robotics-SolidWorks-to-Gazebo.pdf
```

From the complete 3D STEP model:

**ISOLATE THE PART AND CENTER**

- Import to FreeCAD and export one piece (eg. the right wheel) as .dae, but first, it's necessary to be centered:

- View → Workbench → Draft

- Draft → Point → [0,0,0]

- Select the object to move → Move icon → Select the center of the object → Select the point in the origin

**CONVERT TO COLLADA**

- File → Import → Collada