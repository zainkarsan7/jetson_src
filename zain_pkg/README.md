### Practice Package <br>

UR10e with meshes and stuff loaded <br>
download to a workspace, git clone, and build <br>
ros2 launch urdf_tutorial display.launch.py model:=/root/ws_moveit2/src/zain_pkg/urdf/ur10e_xacro_plasma.urdf.xml 
make sure to update the workspace name <br>
<br>
also add this to the settings json in vscode to find the moveit_configs_utils python file

```
"python.analysis.extraPaths": [
        "/root/ws_moveit2/install/moveit_configs_utils/lib/python3.10/site-packages"
    ]
```

## TO DO <br>

* figure out remote desktop
* container for the IM GUI setup from windows 
* start making rviz plugins 
