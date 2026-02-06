# KRSBI-B Robot Meshes

This directory is for robot mesh files (STL/DAE).

## Expected Files

Once CAD models are available, export the following meshes:

- `base_link.stl` - Main robot body
- `wheel.stl` - Omni wheel (used for all 3 wheels)
- `motor_housing.stl` - PG45 motor housing
- `omni_camera.stl` - Omni camera dome
- `front_camera.stl` - Logitech webcam
- `gripper_base.stl` - Gripper base
- `gripper_arm.stl` - Gripper arm (left/right)
- `kicker_housing.stl` - Kicker solenoid housing
- `kicker_plate.stl` - Kicker plate

## Exporting from CAD

When exporting STL files:

1. Use millimeters as units (will be converted to meters in URDF)
2. Center the mesh at the origin
3. Align with ROS coordinate convention:
   - X: Forward
   - Y: Left
   - Z: Up
4. Keep triangle count reasonable (<50k per mesh)

## Using Meshes in URDF

To use meshes instead of primitives, update the URDF:

```xml
<visual>
  <geometry>
    <mesh filename="package://krsbi_description/meshes/base_link.stl" scale="0.001 0.001 0.001"/>
  </geometry>
</visual>
```

Note: Scale of 0.001 converts from mm to meters.
