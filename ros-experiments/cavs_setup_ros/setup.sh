#!/usr/bin/env bash
# One-step setup for the ROS 2 dev container on Linux and macOS.
# Installs Docker and VS Code if they are missing, adds the Dev Containers
# extension, then opens this folder inside the container.
# Safe to run again: steps that are already done are skipped.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXTENSION="ms-vscode-remote.remote-containers"
# Must match "workspaceFolder" in .devcontainer/devcontainer.json
CONTAINER_FOLDER="/workspace"
OS="$(uname -s)"

info() { printf '\n==> %s\n' "$*"; }
fail() { printf '\nERROR: %s\n' "$*" >&2; exit 1; }

if [[ $EUID -eq 0 ]]; then
  fail "Run this as your normal user, not with sudo. It will ask for your password when needed."
fi

find_code() {
  if command -v code >/dev/null 2>&1; then
    command -v code
  elif [[ -x "/Applications/Visual Studio Code.app/Contents/Resources/app/bin/code" ]]; then
    echo "/Applications/Visual Studio Code.app/Contents/Resources/app/bin/code"
  fi
}

# ---------- Linux ----------
setup_linux() {
  if command -v apt-get >/dev/null 2>&1; then
    PKG=apt
    # Ubuntu desktop does not ship curl by default
    sudo apt-get update
    sudo apt-get install -y curl gpg
  elif command -v dnf >/dev/null 2>&1; then
    PKG=dnf
  else
    fail "Unsupported Linux distribution (needs apt or dnf). Install Docker and VS Code yourself, then run this again."
  fi

  if ! command -v docker >/dev/null 2>&1; then
    info "Installing Docker Engine"
    curl -fsSL https://get.docker.com | sudo sh
  fi
  sudo systemctl enable --now docker

  if [[ -z "$(find_code)" ]]; then
    info "Installing VS Code"
    if [[ $PKG == apt ]]; then
      curl -fsSL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor \
        | sudo tee /usr/share/keyrings/microsoft.gpg >/dev/null
      sudo chmod 644 /usr/share/keyrings/microsoft.gpg
      sudo tee /etc/apt/sources.list.d/vscode.sources >/dev/null <<'EOF'
Types: deb
URIs: https://packages.microsoft.com/repos/code
Suites: stable
Components: main
Architectures: amd64,arm64,armhf
Signed-By: /usr/share/keyrings/microsoft.gpg
EOF
      sudo apt-get update
      sudo DEBIAN_FRONTEND=noninteractive apt-get install -y code
    else
      sudo rpm --import https://packages.microsoft.com/keys/microsoft.asc
      sudo tee /etc/yum.repos.d/vscode.repo >/dev/null <<'EOF'
[code]
name=Visual Studio Code
baseurl=https://packages.microsoft.com/yumrepos/vscode
enabled=1
autorefresh=1
type=rpm-md
gpgcheck=1
gpgkey=https://packages.microsoft.com/keys/microsoft.asc
EOF
      sudo dnf install -y code
    fi
  fi

  # VS Code talks to Docker as you, so you need to be in the docker group
  if ! docker info >/dev/null 2>&1; then
    if ! id -nG "$USER" | grep -qw docker; then
      info "Giving your user permission to use Docker"
      sudo usermod -aG docker "$USER"
    fi
    if ! id -nG | grep -qw docker; then
      info "Almost done. Log out and back in (or restart your computer), then run this script again."
      exit 0
    fi
    fail "Docker is installed but not responding. Try restarting your computer, then run this again."
  fi
}

# ---------- macOS ----------
find_brew() {
  if command -v brew >/dev/null 2>&1; then
    command -v brew
  else
    # Apple Silicon and Intel install locations
    for b in /opt/homebrew/bin/brew /usr/local/bin/brew; do
      if [[ -x "$b" ]]; then echo "$b"; return; fi
    done
  fi
}

install_homebrew() {
  info "Installing Homebrew (enter your Mac password when asked)"
  # The unattended installer can't prompt for a password, so ask for it now
  # and keep it fresh while the install (and Xcode tools download) runs
  sudo -v
  while true; do sudo -n true; sleep 50; done 2>/dev/null &
  local keepalive=$!
  NONINTERACTIVE=1 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" \
    || { kill "$keepalive"; fail "Homebrew install failed. Your Mac user must be an administrator."; }
  kill "$keepalive"
}

setup_macos() {
  if [[ -z "$(find_brew)" ]]; then
    install_homebrew
  fi
  local brew
  brew="$(find_brew)"
  [[ -n "$brew" ]] || fail "Homebrew was installed but could not be found. Open a new terminal and run this again."
  eval "$("$brew" shellenv)"
  # Make brew available in future terminals too
  if ! grep -qs 'brew shellenv' ~/.zprofile; then
    echo "eval \"\$($brew shellenv)\"" >> ~/.zprofile
  fi

  if ! command -v docker >/dev/null 2>&1 && [[ ! -d /Applications/Docker.app ]]; then
    info "Installing Docker Desktop"
    brew install --cask docker-desktop || brew install --cask docker
  fi

  if [[ -z "$(find_code)" ]]; then
    info "Installing VS Code"
    brew install --cask visual-studio-code
  fi

  if ! docker info >/dev/null 2>&1; then
    info "Starting Docker Desktop (the first time, accept the terms in the Docker window)"
    open -a Docker
    for _ in $(seq 60); do
      sleep 5
      docker info >/dev/null 2>&1 && break
    done
    docker info >/dev/null 2>&1 \
      || fail "Docker did not start. Open Docker Desktop, wait until it says 'Engine running', then run this again."
  fi
}

case "$OS" in
  Linux) setup_linux ;;
  Darwin) setup_macos ;;
  *) fail "Unsupported system: $OS. On Windows, double-click setup.bat instead." ;;
esac

CODE="$(find_code)"
[[ -n "$CODE" ]] || fail "VS Code was installed but the 'code' command was not found. Open a new terminal and run this again."

info "Installing the Dev Containers extension"
"$CODE" --install-extension "$EXTENSION"

info "Opening the project in the dev container (the first build takes several minutes)"
# VS Code identifies a dev container folder by its host path, hex-encoded
hex_path="$(printf '%s' "$REPO_DIR" | od -An -v -tx1 | tr -d ' \n')"
"$CODE" --folder-uri "vscode-remote://dev-container+${hex_path}${CONTAINER_FOLDER}"
