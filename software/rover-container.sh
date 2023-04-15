sudo docker build -t ros2 .
sudo docker run -dit -v /dev:/dev --privileged --name rover2 ros2
