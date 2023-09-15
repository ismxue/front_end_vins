# front_end_vins

- Build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# download files
cd ..
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```
- Run

```
# Terminal 1

# split extraction and match section
roslaunch feature_tracker plfeature_tracker_split.launch

or

# combine extraction and match section
roslaunch feature_tracker plfeature_tracker.launch
```

```
# Terminal 2
rosbag play src/sp-sold2-vins/Trajactory/MH_01_easy.bag

```
- Test (Optional)

```
cd ~/catkin_ws

./runtime_split.py
roslaunch feature_tracker plfeature_tracker_split.launch > pl_res_split.txt
rosbag play src/sp-sold2-vins/Trajactory/MH_01_easy.bag
grep -o "current frame" pl_res_split.txt | wc -l

or

./runtime_baseline.py
roslaunch feature_tracker plfeature_tracker.launch > pl_res_baseline.txt
rosbag play src/sp-sold2-vins/Trajactory/MH_01_easy.bag
grep -o "current frame" pl_res_baseline.txt | wc -l

```


![Untitled.png](https://s2.loli.net/2023/09/15/QamFuk12WEtdli3.png)
![Untitled1.png](https://s2.loli.net/2023/09/15/1ygi4tdNCR89YzA.png)

