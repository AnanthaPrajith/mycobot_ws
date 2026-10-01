"""Make workspace ROS packages visible to xacrodoc in lab notebooks."""

from xacrodoc import packages

packages.update_package_cache(
    {
        "mycobot_description": "/home/corobot/mycobot_ws/src/mycobot_description",
    }
)
