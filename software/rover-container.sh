sudo docker build -t ros2 .
sudo docker run -dit -v /dev:/dev --privileged --net=host --name rover2 ros2
