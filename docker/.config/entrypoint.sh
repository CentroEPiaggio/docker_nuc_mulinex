#!/bin/bash
set -e

# Setup environment
source $HOME/.bashrc

# Give the container user a login password for SSH.
printf '%s:%s\n' "$USER" "${SSH_PASSWORD:-mulinex}" | sudo chpasswd

# Start sshd so the container can be reached with ssh on port 2222.
sudo mkdir -p /run/sshd
sudo ssh-keygen -A
if ! pgrep -x sshd >/dev/null; then
	sudo /usr/sbin/sshd
fi

exec "$@"
