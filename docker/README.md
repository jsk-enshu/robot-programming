# Dockerfile for jsk-enshu

## For MAC-M1 user

First, please install [Docker Desktop](https://www.docker.com/products/docker-desktop) and launch it.

Next, clone this repository and build docker.

```
$ git clone https://github.com/jsk-enshu/robot-programming
$ cd robot-programming
$ git remote add iory https://github.com/iory/robot-programming
$ git fetch iory
$ git checkout -b docker iory/docker
$ cd docker
$ ./build.sh
```

After that, run the docker container.

```
$ ./run.sh
```

You can access inside the container by accessing the following url.

http://localhost:8080/vnc_auto.html

Now you can start the exercise!

