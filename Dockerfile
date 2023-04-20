FROM ros:foxy

WORKDIR "/root/VEXU_GHOST"

RUN sudo apt-get update

COPY ./scripts/install_dependencies.sh /

RUN chmod +x /install_dependencies.sh

RUN bash /install_dependencies.sh

RUN sudo apt-get install -y ros-foxy-ros2-control ros-foxy-teleop-twist-keyboard tmux neovim gcc-arm-none-eabi

RUN python3 -m pip install pros-cli
