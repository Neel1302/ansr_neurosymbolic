#### ADK & Airsim_keystore
* Information related to the ADK can be found in the `ta3-documentation` [repository](https://github.com/darpa-ansr/ta3-documentation/).
* The [README.docx](https://github.com/darpa-ansr/ta3-documentation/blob/main/README.docx) contains useful information
  although we will be running the software in a different way than described
* One important thing to note is that the `ADK` is running [ROS2's security stack](https://docs.ros.org/en/humble/Tutorials/Advanced/Security/Introducing-ros2-security.html)
  which restricts what actions can be taken with specific topics and requires a keystore to be setup. The `ADK`'s keystore
  can be found in the `ta3-documentation` repository [here](https://github.com/darpa-ansr/ta3-documentation/tree/main/sros2/airsim_keystore)
  and will be mounted inside the `development-env` container.