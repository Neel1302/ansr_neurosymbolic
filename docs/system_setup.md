### System Requirements

Your machine requires an Nvidia GPU with Cuda, Docker, and the Nvidia-Container-Toolkit installed. This repository
assumes that you are working on a Linux machine with Ubuntu 20.04 or 22.04 (although other Linux distros should work fine).

#### Installing Docker
* The `adk` is provided via a `docker image` and we will be running they system via `docker compose`.
* Docker installation instructions can be found [here](https://docs.docker.com/engine/install/ubuntu/)
* Follow the [post-installation instructions](https://docs.docker.com/engine/install/linux-postinstall/) to ensure that 
  docker does not need to be run with `sudo`
* To test that docker is properly installed run: `docker run hello-world`

#### Installing Cuda
* Cuda drivers are required so that your machine can interact with your Nvidia GPU.
* If you do not already have Cuda installed, detailed instructions can be found
  [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/#prepare-ubuntu)
  and simplified instructions can be found [here](https://developer.nvidia.com/cuda-downloads).
* To test that Cuda is properly installed run: `nvidia-smi`

#### Installing the Nvidia-Container-Toolkit
* The Nvidia Container Toolkit allows docker to leverage Cuda.
* Installation instructions can be found [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)
* To test that the Nvidia Container Toolkit is properly installed run: `docker run --rm --gpus all ubuntu nvidia-smi`
