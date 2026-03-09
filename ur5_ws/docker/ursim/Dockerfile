# https://hub.docker.com/r/universalrobots/ursim_cb3
FROM universalrobots/ursim_cb3:3.15

# Install the external control URCap (required for ROS2)
# Reference:
# - https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/ursim_docker.html#external-control
# - https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap
ARG URCAP_VERSION=1.0.5
ENV URCAP_VERSION=${URCAP_VERSION}
RUN curl -fSL -o /urcaps/externalcontrol-${URCAP_VERSION}.jar \
    "https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar"
