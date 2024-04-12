#### Modifying Permissions on the Mission Briefing Files
* Mission briefing files are built into the `adk` image under `/Scenario`. I believe that:
  * `/Scenario/Perception/AreaSearch` contains necessary configuration and description files. These are paired based
    on the file names. Inside the chosen configuration file, you can find the associated path file name.
  * `/Scenario/SafeManeuverRoutes` contains the path files
* When you run the `adk` with the `--mission_thread=` option it will write the necessary scenario files to the 
  `mission_briefing` folder (unless the `--use-mission_briefing` command is also used).