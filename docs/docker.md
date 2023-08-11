# Using Docker with MJBots
We have compiled a compatible docker image with all the necessary components to compile the Kodlab MJBots SDK, and communicate with the Pi with LCM. This Docker image should work with any operating system that Docker supports, however this tutorial assumes your operating system has BASH commands, and git installed.

## Repository Setup
Clone the [kodlab_mjbots_sdk repository](https://github.com/KodlabPenn/kodlab_mjbots_sdk), `cd` into the folder, and run
`git submodule init
git submodule update`

## Installing Docker
Download the docker application from the [Docker Website](https://docs.docker.com/get-docker/). If you are unfamiliar with Docker, read through their [getting started section](https://docs.docker.com/get-started/)  to gain an understanding of the underlying structure of Docker. 

## Setting up Docker for the SDK
The first step to setting up Docker after install, is to acquire the correct Docker Image. There are two ways to do this. You can either clone the **kodlab-cross** image from Docker Hub, or build the **kodlab-cross** Docker Image from source. 

### Cloning the image from Docker Hub

You can download the **kodlab-cross** image from the Docker Hub using the following command:

`docker image pull masondmitchell/kodlab_mjbots_cross_compile`

and that's it! You should be able to run containers with fully set up environments to build the codebase. 

However, if you run into problems during this step, or you want to edit the environment locally, then building from source might be the better option. 

### Building from Source
**WARNING:** Building the image from source can take over an hour. 

To build from source, go into the `Docker` folder with 
`cd Docker` 
then run the `build.sh` script with
`./build.sh` 
or instead run the terminal command:
`docker build --progress tty -t masondmitchell/kodlab_mjbots_cross_compile .`
which should create the **kodlab-cross** image using the included Dockerfile. This will take a while.

## Container and LCM Setup with Docker

The next setup step is usually to make lcm by running the `./scripts/make_lcm.sh` script. However, this command should be run **inside the docker container**. 

So, we must open a docker container using the **kodlab-cross** image. This docker container does not have any users, so the home directory is `/root`. This is where we want to mount our [kodlab_mjbots_sdk repository](https://github.com/KodlabPenn/kodlab_mjbots_sdk), as well as our `raspberrypi/rootfs` folder (**NEEDS HOSTED ON GITHUB**). To do this, the run command to open the container needs an [absolute path](https://www.computerhope.com/issues/ch001708.htm) to both of these folders that are stored on your own machine. You can find these by going into the `kodlab_mjbots_sdk` and `raspberrypi` folders in terminal, and running `pwd`. Replace the `<KODLAB_MJBOTS_SDK ABSOLUTE PATH>` and `<RASPBERRYPI ABSOLUTE PATH>` placeholders in the command below and run it.

`docker run --platform=linux/amd64 --mount type=bind,source=<KODLAB_MJBOTS_SDK ABSOLUTE PATH>,target=/root/kodlab_mjbots_sdk --mount type=bind,source=<RASPBERRYPI ABSOLUTE PATH>,target=/root/raspberrypi -it masondmitchell/kodlab_mjbots_cross_compile`

This will spin up a new container with the mounted folders in their correct target locations, and open an interactive bash interface in your terminal window. 

You now need to run the `make_lcm.sh` script while inside the container in order to configure lcm correctly. You can do this by running the command:
`./root/kodlab_mjbots_sdk/scripts/make_lcm.sh`.

You can now close the container by typing `exit` in the command window. 

## Building the SDK with Docker

You should now be able to compile the SDK using docker. To do this run the command

`docker run --rm --platform=linux/amd64 --mount type=bind,source=<KODLAB_MJBOTS_SDK ABSOLUTE PATH> --mount type=bind,source=<RASPBERRYPI ABSOLUTE PATH>,target=/root/raspberrypi masondmitchell/kodlab_mjbots_cross_compile bash -c "cd kodlab_mjbots_sdk && rm -R build/ && mkdir build && cd build/ && cmake .. -DCMAKE_TOOLCHAIN_FILE=/root/kodlab_mjbots_sdk/cmake/pi.cmake && make"`

Docker will spin up a new container using the **kodlab-cross** docker image, and run the clean build command found in [Usage](https://kodlab-mjbots-sdk.readthedocs.io/en/latest/usage/#building).

## Using LCM With Docker
NOTE: Need to talk about `Docker/run_latest.sh` to open multiple terminal windows from the same container. (Can maybe set up a script to do this automatically?)

There are some notable differences when using LCM with Docker. The main one being a build error when attempting to run `lcm-logger`. To fix this, run `sudo ldconfig -v` before establishing the lcm tunnel to resolve this issue. 

## VSCode with Docker
 Coming soon...
