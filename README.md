# Cartographer with F1tenth simulator

This package provides cartographer launches with f1tenth simulation system.


F1tenth simulator

https://github.com/f1tenth/f1tenth_gym_ros


Cartographer Ros

https://github.com/cartographer-project/cartographer







# Problems 

1. 기존 F1tenth system의 tf link로는 Cartographer의 pose optimization을 이용하기 어렵다.
  다음과 같이 되어야 하나 : (global frame) -> (odomerty frame) -> (base frame) -> ...
  f1tenth 에서는 : global frame (map) -> (pose) -> (base_frame) 으로 tf tree가 이루어짐.


2. Cartographer에서는 slam을 진행하기 위해 로봇의 urdf 파일을 작성하여 로봇의 structure을 구성하여야 한다. 


# Solution

1. f1tenth system 의 tf을 유지하되,  해당 tf을 적절히 복제하여 Cartographer의 frame 을 설정한다.
2. urdf 파일을 위 1. 에서 복제한 tf 으로 설정한다.

# Usage
--------

```bash
cd launch
ros2 launch 
```
