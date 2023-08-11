# Using Docker with MJBots
We have compiled a compatible docker image with all the necessary components to compile the Kodlab MJBots SDK, and communicate with the Pi with LCM. This Docker image should work with any operating system that Docker supports, however this tutorial assumes your operating system has BASH commands.

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
which should create the **kodlab-cross** image using the included Dockerfile. This is the command that will take a while.

and running the `git submodule init` and `git submodule update` commands, the next setup step is usually to make lcm by running the `./scrips/make_lcm.sh` script. However, this command should be run inside the docker container. To do this, run the `./Docker/run.sh` executable which will spin up a new container and open a bash interface 

## Building the SDK with Docker

After completing the setup above, you should now be able to compile the SDK using docker. To do this run the `Docker/compile.sh` executable in the repository, using `./Docker/compile.sh` while in the `kodlab_mjbots_sdk` folder. Docker will spin up a new container using the **kodlab-cross** docker image, and run the clean build command found in [Usage](https://kodlab-mjbots-sdk.readthedocs.io/en/latest/usage/#building).

## LCM With Docker
NOTE: Need to talk about `Docker/run_latest.sh` to open multiple terminal windows from the same container. (Can maybe set up a script to do this automatically?)

There are some notable differences when using LCM with Docker. The main one being a build error when attempting to run `lcm-logger`. To fix this, run `sudo ldconfig -v` before establishing the lcm tunnel to resolve this issue. 

## CLion with Docker
 Coming soon...
