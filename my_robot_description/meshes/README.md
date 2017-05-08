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

- Select the object to move → Move the pice by hand (not the best option but it's fast and useful)

**CONVERT TO COLLADA**

- File → Export → Collada (.dae)

**RESIZE WITH Blender**

- File → Import → Collada (.dae)

- Pull out the right-tab in blender (look for a plus sign near the upper right of the render window). Under the Dimensions section of this tab, divide the x,y,z components by 1000. **NOTE:** Can be applied to the _scale_ section.

- File → Export → Collada (.dae) → Export COLLADA