# Overview
This repo is intended to be used as part of the second DARPA ANSR dry run activity, with the goal of ensuring that performers are familiar with TA4 expectations for building and pushing Docker images that are compatible with TA3 ADK. It requires a Linux environment capable of running the ADK Airsim docker images (preferably, Ubuntu 20.04 or 22.04 system with an nvidia GPU).
 
Questions should be posted to DAPRA ANSR #evaluation channel, or to the Github issues for this repo.

### Extra Documentation
More advanced documentation can be found in the `docs/` directory.

### Cloning the Repository
The first thing you need to do is to clone this repository and its submodules.

```
git clone https://github.com/darpa-ansr/hello-world-template.git --recurse-submodules
```

### Environment Setup
There are some steps that we need to take to ensure that the adk and our node runs successfully.

1. Every time we open a terminal that will run the adk, we need to run `xhost +local:root`
2. We need to ensure the `adk/mission_briefing` and `adk/output` directory have the proper permissions so that they can 
   be written to. Run: `chmod -R 777 adk/mission_briefing` and `chmod -R 777 adk/output`

### An example development flow
Say we want to modify the hello world python package that we have mounted in container, and we want to test it using the `adk` image. This requires three terminals.
1. In terminal 1, we launch the development container: 
    ```
    docker compose run development-env
    ```
   This is going to launch a bash shell inside the `development-env` container due to the `command` in the `docker-compose.yml` file.
   This command is passed into our image `entrypoint` defined over in `docker/ros_entrypoint`. Note that we handle sourcing the workspace that was
   built as part of the image building process (see the `Dockerfile`).
2. Locally, we make our changes to `./src/ansr_hello_world_py` using our favorite IDE.
3. In terminal 1, (which is a shell in the container now), we build the package and run it: 
    ```
    colcon build --packages-select ansr_hello_world_py
    ros2 run ansr_hello_world_py publisher
    ```
4. Now, it is time to test. In terminal 2, we launch the adk: 
    ```
    docker compose run adk
    ```
   You are going to want to run the adk in a fresh container for each run as we have been running into some issues with
   container state if you try to manually run `adk_ros_entry.sh`. So make sure you run `docker container prune` every 
   once in a while to avoid too many dangling containers.
5. Now we can check if the publisher is actually working. In the terminal 3, we are going to run commands in the `development-env` container:
    ```
    docker compose exec development-env bash
    (in the container) source install/setup.bash
    (in the container) ros2 topic echo adk_node/input/perception
    ```
    We should see the outputs of the hello world node!

6. Lastly, say the node is not performing as expected. We don't have to bring down everything! We can simply stop the execution of the node in terminal 1, stop the ADK node in terminal 2, make the changes in the IDE, and then re-run from step 3. 

# For the Second Dry Run Evaluation

We have updated the original `hello-world-template` to include an object detection algorithm using the HuggingFace framework.
To do this, we added the necessary dependencies into the Dockerfile and burnt the model weights into the image as part of the building process,
so they would not need to be fetched later. We have updated the `/adk_node/input/perception` message to publish the output of the model and 
we are publishing the version of the dependency on the `/adk_node/dependency` topic to prove that the dependency has been successfully added.

Your task is to do something similar: add some kind of dependency into the docker image and show that it was added successfully.

**We do not need a functioning component**.

If you are working in python, you could build or add packages, import them, and share some kind of version information (like we have done).

If you are working in C++, you could write a small hello-world style node that includes a dependency's header file and publish its name or something about the dependency.


## Building
The build process is very simple. 

```
docker build -t ansr-hello-world .
```
   

## Testing
Your image can be tested by using the deploy docker compose file. This does NOT mount local volumes and only relies on 
the packages built into the container. This is similar to how we will be evaluating your images, so make sure it works!

**NOTE: we expect all necessary files and commands for your system to be run to be built into the image itself so we can
  add it as a service to our evaluation `docker-compose` without needing changes**

  ```
  docker compose -f docker-compose.deploy.yml up
  ```
  If everything works, you should see no errors! We can check if the publisher is working by running another set of commands in the `deployment` container
  
  ```
  docker compose exec deployment bash
  (in the container) source install/setup.bash
  (in the container) ros2 topic echo adk_node/input/perception
  ```
  
  We should see the outputs of the hello world node!

## Pushing
Once the image has been built and tested, you can push your image.

First, the image must be tagged with your performer name (`cmu`, `collins`,`monash`, `sri`, `berkeley`, `ucf`, `ucla`, or `vanderbilt`).

**Note: In order to be evaluated it must be named in this format.**

```
docker tag ansr-hello-world darpaansr.azurecr.io/<performer-name>:hello-world-0.0.2
```

Then, you can push the image.
```
docker push darpaansr.azurecr.io/<performer-name>:hello-world-0.0.2
```
