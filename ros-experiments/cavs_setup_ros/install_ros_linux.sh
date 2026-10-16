#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_USER="${SUDO_USER:-$USER}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"

sudo apt remove $(dpkg --get-selections docker.io docker-compose docker-compose-v2 docker-doc docker-buildx podman-docker containerd runc | cut -f1) -y


sudo apt update
sudo apt install ca-certificates curl -y
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update -y


sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y

sudo systemctl enable docker

sudo systemctl start docker

sudo docker run hello-world

sudo usermod -aG docker "$TARGET_USER"
sudo -u "$TARGET_USER" mkdir -p "$TARGET_HOME/ros2_ws/src"


sudo apt install git -y

sudo docker build -t ros2-kilted-dev:latest "$SCRIPT_DIR"

echo
echo "Done. Log out and back in (or run 'newgrp docker'), then start the dev container with:"
echo "  docker run -it --rm --network host --ipc host -v ~/ros2_ws:/workspace ros2-kilted-dev:latest"