# sw2_ccv_manipulation

## bridgeの仕方

docker内でgazeboを起動し、sq2_ccvをsq2_ccv_manipulation/twist_ccvで動かしたいとき、

ターミナル1で
```bash
r1
roscore
```

ターミナル2でbridge起動

```bash
r1 #r1が先
r2d
ros2 run ros1_bridge dynamic_bridge
```

ターミナル3でdocker起動
```bash
$ ./run_nvidia_docker.sh

cd catkin_ws/
catkin_make; source devel/setup.bash
roslaunch sq2_ccv_description gazebo.launch
```

ターミナル4でpublisher起動

```bash
cd ~/ccv_ws
r2d
rs
ros2 run sq2_ccv_manipulation twist_ccv 
```