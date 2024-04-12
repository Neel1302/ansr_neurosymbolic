#### ANSR Registry Credentials
* All `ANSR` related docker images are stored in the `ANSR` Docker registry: `darpaansr.azurecr.io`. This registry is
  hosted on Microsoft's Azure platform. You will receive a username and password that you will be using to access this
  docker registry.
* Run: `docker login -u <username> -p <password> darpaansr.azurecr.io` to log in to the `ANSR` registry.
* Run: `docker pull darpaansr.azurecr.io/adk:latest` to pull the latest `ADK` image
