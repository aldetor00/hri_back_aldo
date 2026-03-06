primo terminale
```
/usr/local/webots/webots
```

secondo terminale
```cd ~/Scaricati
./booster-runner-full-webots-7dof_arms-0.0.3.run
```

terzo
```cd ~/booster_ws/src/booster_robotics_sdk/build 
./b1_loco_example_client 127.0.0.1
```

quarto
```
cd ~/Scrivania/booster_robotics_sdk-main/build
 sudo ./b1_low_level_subscriber lo

```
quinto
```
ros2 topic list
```
aggiungi questo in fondo al  bash 
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
```
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
gedit ~/.bashrc
source ~/.bashrc

```

```
```



This project is licensed under the Apache License, Version 2.0. See the LICENSE file for details.

This project uses the following third-party libraries:
- fastDDS (Apache License 2.0)
- pybind11 (BSD 3-Clause License)
- pybind11-stubgen (MIT License)
