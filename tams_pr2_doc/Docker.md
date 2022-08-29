Docker works similar to a virtual machine, it is usually used to isolate applications from the underling hardware and the operating system, also called "containerization". In our case we want to run an ubuntu 16.04. with ros-kinetic, without changing our current operating system. The most important terminology of docker are "images" and "containers". Images specify the content of our isolated application, these images are permanently stored on the host system. If we start in instance of an image we call it "container", we can start multiple containers of the same image. But if we change anything in a running container, it will not be adopted to the image. We generate our image with a Dockerfile, this is like a construction manual for our custom application. We start with a bare ubuntu 16.04. image and install all the packages we need, we can also create users or directories or any other thing we can do on a normal system within the Dockerfile.  
Docker is usually used for cloud computing and providing a dynamic amount of applications. In our case we misuse docker in some way. We don't want to upgrade the whole pr2 to a new os, so we keep the current ubuntu 14.04 and run a docker with 16.04 to use kinetic. In more detail, we have one Dockerfile that can be used for c1 and c2, due to the overlay and the problem that docker is not working properly with nfs mounted disks we can't share the image on both machines. Changing anything in the image means we need to build it on c1 and c2 separately. On the base station we have two separate disks, one with ubuntu 14.04. and one with 16.04. (IN WORK), so we don't need a docker here. We can just boot the other os and run the pr2 in indigo/kinetic if we want. For the head, we have a custom docker image different to the one on c1 and c2, it might happen that we upgrade to an ubuntu 16.04. in the future.


-----


#### How to start the robot
You can connect the the c1 docker container similar to c1 with ssh. With `ssh c1 -p 122` or with the aliases `c1-docker` or `ssh docker-c1`. The same applies to c2. After connecting to the container, you are located in the home directory of the docker container, this looks and works just the same as your host shell. Now you can continue the same way as usual with `roslaunch tams_pr2_bringup trixi.launch` and so on.

#### How to develop
Everything in the home directory is shared with the host system, you can find it at `~/pr2_docker/home/`. This directory will not reverse after a restart of the robot (or the container). You need to compile and run your code inside the docker container, remember that the host system still runs with ubuntu 14.04. and ros-indigo. Only within the container you have a 16.04. Ubuntu and ros-kinetic. To be save, you should never connect to the c1 host, but to the c1-docker during the development. Within the container and your workspace you can use `catkin_make` or `catkin build` as usual.


#### How to install packages and making updates
If you install packages in the running container, they will be gone after a restart. To install them permanently you need to add an entry in the Dockerfile. You find it at `~/pr2_docker/docker/Dockerfile`, add a new line with `RUN apt-get install -y package` or extent the available lists if your package fits there. To apply the changes you need to build the image again. You can also commit the changes you've done on the container, see "link todo". But remember that the containers on c1 and c2 are not shared, you need to commit the changes for both containers. If you forget the entry in the Dockerfile, your packages disappear when you rebuild the image. For updates, we just need to rebuild our image, it will use the newest available packages during the build.

#### How to shutdown the robot
To shutdown the robot you need to connect to the host system of c1 and use `sudo pr2-shutdown`, this will not work in the docker container. Remember that only the home directory is permanent, all other changes you did to the container are gone on the next restart.

#### Which folders do we share (permanent)
Shared folders are located on the host system and are shared with the running container. That way the files are permanent and don't disappear after a restart, but they are also not saved within the docker image. The most important folder we share is the home directory, here are among others our ros workspaces and the bashrc. Another important folder is the .ssh folder we share with the host, that way we can use the ssh keys, but don't need to save them in the image which could be a security problem. We also share some custom pr2 links that points to different sensors.

#### Build Docker image
We want to build our image using a Dockerfile, the Dockerfile is like a construction manual of our image or our custom ubuntu 16.04. operating system. To build the image we need to call `docker build --network=host -t "pr2_kinetic:latest" -f path_to_dockerfile .`. Every time we want to make changes of the image like updates or new packages we need to rebuild it. We can't share the images of c1 and c2 so we need to build it separately on both machines.

#### Start or restart a container
If there is a runnable image, the container will start automatically on startup. A docker container is usually started with the docker run command, in our case we need to add a lot of parameter, which makes the command very long. Thats the reason we use docker-compose, this is similar to the Dockerfile, not a construction manual for building the image, but for starting it. Here you can specify a lot of things, like shared volumes, used ports, privileges and so on. Sometimes it is necessary to stop or restart a running container, this can be done from the host with `docker stop container_name` to stop the container and then using `docker-compose up -f compose_file` to start the container again.

#### Autostart container on robot startup

By default, docker will *stop* containers on computer shutdown and you can continue them by *start*ing them after booting again.
On the PR2 we automated this via an init file `/etc/init/docker-compose-pr2.conf`.

#### Commit changes of the container
Try to avoid this!  
If you changed anything in the container, like installing a new package, you can commit it and the change is kept after a restart of the robot or the container. That way you don't need to rebuild the image if you only installed one package, but keep in mind, if you forget to add the change to the Dockerfile, it disappears after the next rebuild of the image. You can commit changes with: `docker commit container_name image_name:tag` in our case it is `docker commit pr2_kinetic_docker pr2_kinetic`

#### Useful commands
Build a new image (separately on c1 and c2):  
`docker build --network=host -t "pr2_kinetic:latest" -f ~/pr2_docker/docker/Dockerfile ~/pr2_docker/docker/`

Stop the running container:  
`docker stop pr2_kinetic_docker`

Create a new container:  
`docker-compose -f ~/pr2_docker/docker/docker-compose.yml up -d`

Delete all images:  
`docker rmi $(docker images -q) -f`

Delete all containers:  
`docker system prune -a`
