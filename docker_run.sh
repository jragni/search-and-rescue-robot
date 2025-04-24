docker run -it --rm --user ubuntu --ipc=host --uts=host --network=host \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY \ 
-v $PWD:/home/ubuntu/search-and-rescue-robot \
-v /dev/:/dev \
--device-cgroup-rule='c *:* rmw' ros_2_image