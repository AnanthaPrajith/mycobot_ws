# mycobot_description

A ROS 2 Python package that contains the .urdf and .dae files to describe every elephant robotics device.  
I added [**mycobot_irf_scene.urdf**](../mycobot_description/urdf/mycobot_280_pi/mycobot_irf_scene.urdf).

---

### `mycobot_irf_scene.urdf`

This .urdf file extends the standard mycobot 280pi .urdf file by adding a collision cylinder for the pump head (the original mesh is corrupt) as well as collision geometries for the surroundings at the workplace. Those include the table, the camera and camera mount and the power bar behind the table. [**mycobot_irf_scene.urdf**](../mycobot_description/urdf/mycobot_280_pi/mycobot_irf_scene.urdf) is useful for collision avoidance while trajectory planning in moveit.  
Camera collision geometry as an example:
```python
[...]
  <link name="env_camera">
    <visual>
      <geometry><cylinder radius="0.05" length="0.05"/></geometry>
      <material name="black_color"><color rgba="0.1 0.1 0.1 1.0"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="0.05" length="0.05"/></geometry>
    </collision>
  </link>
  <joint name="world_to_camera" type="fixed">
    <parent link="world"/>
    <child link="env_camera"/>
    <origin xyz="0.16 0 0.39" rpy="0 0 0"/>
  </joint>
[...]