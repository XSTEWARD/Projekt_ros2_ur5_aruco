# Bazujemy na oficjalnym obrazie ROS 2 Humble
FROM ros:humble

# Ustawiamy zmienne srodowiskowe
ENV PYTHONUNBUFFERE=1
#Instalacja pakietow systemowych i sterownikow ROS
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-usb-cam \
    ros-humble-ur-robot-driver \
    ros-humble-ur-client-library \
    ros-humble-rqt-image-view \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

#Instalacja bibliotek Python (OpenCV)
RUN pip3 install opencv-contrib-python numpy

#Tworzenie przestrzeni roboczej
WORKDIR /root/ros2_ur5_aruco_ws/src

#Automatyczne source'owanie
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Komenda domyslna
CMD ["bash"]
