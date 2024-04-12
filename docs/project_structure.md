### Project Structure
* `adk`: contains any files related to the adk container. This currently includes:
  * `mission_briefing`: where the `adk` will write scenario files 
  * `output`: the location where `adk` will write any output
* `docker`: contains files necessary for creating the docker development and deployment images
  * `02-rossource.sh`: have bash source our ros `setup.bash` files
  * `ros_entrypoint.sh`: the script that runs by default whenever the `environment-dev` container is launched. The
    current entrypoint sources the local workspace `setup.bash` and then runs the given command. This can be overriden 
    in `docker-compose.yml` via the `entrypoint` keyword.
  * `run_node.sh`: the script that runs when the `development-env` container is launched
* `src`: development packages
* `ta3-documentation`: ta3-documentation repository
* `docker-compose.yml`: docker orchestration file
* `Dockerfile`: Dockerfile for building the `environment-dev` image
* `docker-compose.deploy.yml`: docker orchestration file for testing the image you build