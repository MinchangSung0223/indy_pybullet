# panda_pybullet_with_ros
```bash
chmod 777 delete_files.sh
chmod 777 move_files.sh
./delete_files.sh
python calibration_simulation.py
./move_files.sh


```
# matlab
```bash
 start Demo2.m
```

# panda.urdf
```bash
gedit Panda/panda.urdf
```
시뮬레이션 카메라 위치 변경
```
  <joint name="panda_camera" type="fixed">
    <parent link="panda_hand"/>
    <child link="panda_camera"/>
    <origin rpy="0 0 0" xyz="0.1 0 0"/>
  </joint>
```
# indy_pybullet
python test.py
