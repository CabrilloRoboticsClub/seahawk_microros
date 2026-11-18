FROM ros:lyrical
ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Add stuff for MicroROS 
RUN apt update -y && \
    apt install -y build-essential cmake gcc-arm-none-eabi sudo \
        libnewlib-arm-none-eabi doxygen git python3 python3-pip ssh

# Delete user if it exists in container
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

ENV SHELL=/bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]