#!/bin/bash
set -e

# Setup environment
source $HOME/.bashrc

if [ "${ENABLE_SSH:-1}" = "1" ]; then
	SSH_PORT="${SSH_PORT:-2222}"
	SSH_USER="${SSH_USER:-mulinex}"
	SSH_PASSWORD="${SSH_PASSWORD:-}"

	if [ -z "$SSH_PASSWORD" ]; then
		echo "SSH_PASSWORD is required when ENABLE_SSH=1" >&2
		exit 1
	fi

	sudo sh -c "cat > /etc/ssh/sshd_config.d/mulinex.conf" <<EOF
PasswordAuthentication yes
PermitRootLogin no
PubkeyAuthentication yes
AllowUsers ${SSH_USER}
EOF

	# Password is applied on startup so it can come from runtime env vars.
	echo "$SSH_USER:$SSH_PASSWORD" | sudo chpasswd

	# Ensure mulinex user gets ROS transport env vars automatically on login.
	if ! sudo grep -q "# Mulinex ROS vars" "/home/${SSH_USER}/.bashrc" 2>/dev/null; then
		sudo sh -c "cat >> /home/${SSH_USER}/.bashrc" <<EOF

# Mulinex ROS vars
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-10}
export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
EOF
		sudo chown "${SSH_USER}:${SSH_USER}" "/home/${SSH_USER}/.bashrc"
	fi
	sudo mkdir -p /run/sshd
	sudo /usr/sbin/sshd -p "$SSH_PORT"
fi

if [ "$#" -gt 0 ]; then
	exec "$@"
fi

exec bash
