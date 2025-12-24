# Projekt_ros2_ur5_aruco
Projekt sterowania robotem UR5 za pomocą znaczników ArUco w ROS2

Projekt sterowania manipulatorem przemysłowym UR5 za pomocą systemu wizyjnego opartego na znacznikach ArUco. Projekt zrealizowany w środowisku ROS2.

Prezentacja działania

TUTAJ FILM!!!!!!!!!!!!!!!!

Funkcjonalności
- Detekcja ArUco: Wykrywanie znacznika 4x4 i jego pozycji na obrazie.
- Logika sterowania:
    * GÓRA: Uniesienie ramienia robota.
    * DÓŁ: Opuszczenie ramienia.
    * STOP: Zatrzymanie robota w aktualnej pozycji (gdy znacznik jest w strefie środkowej).
    * Pamięć ruchu: Po utracie widoczności znacznika robot kontynuuje ostatnie polecenie.

Wymagania
-  Ubuntu 22.04 
- ROS 2
- Kamera USB
- Biblioteki Python: opencv-contrib-python, numpy

Instalacja
1.  Sklonuj repozytorium do katalogu domowego:
2.  Zainstaluj sterowniki robota i kamery:
    - sudo apt update
    - sudo apt install ros-humble-ur-robot-driver ros-humble-ur-client-library ros-humble-usb-cam
3.  Zbuduj pakiet:
    - cd ~/ros2-ur5-aruco
    - colcon build
    - source install/setup.bash

Uruchomienie

System wymaga uruchomienia w trzech osobnych terminalach:

Terminal 1: Sterownik kamery
ros2 run usb_cam usb_cam_node_exe

Terminal 2: Symulacja Robota UR5
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true initial_joint_controller:=joint_trajectory_controller robot_ip:=192.168.56.101

Terminal 3: Węzeł sterujący
ros2 run aruco_controller aruco_node

Instrukcja obsługi
1. Uruchom wszystkie terminale.
2. W oknie kamery zobaczysz dwie niebieskie linie wyznaczające strefę STOP
3. Pokaż znacznik ArUco
   - Powyżej górnej linii: Robot unosi ramię.
   - Poniżej dolnej linii: Robot opuszcza ramię.
   - Pomiędzy liniami: Robot zatrzymuje się natychmiast.

Autor: Nikodem Michałowicz
