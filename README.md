# Docker NUC Mulinex

This repository contains the Docker configuration for the NUC of the Mulinex project.

## Initial Setup of the Machine

For the initial setup of the NUC and of the Raspberry, refer to the **omnicar** section in [mulinex_guide](https://github.com/CentroEPiaggio/mulinex_guide).

## Preliminaries

Clone the repository with the `--recursive` option to also clone the submodules!

Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine).
You can follow the installation method through `apt`.
Note that it makes you verify the installation by running `sudo docker run hello-world`.
It is better to avoid running this command with `sudo` and instead follow the post installation steps first and then run the command without `sudo`.

Follow with the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for Linux.
This will allow you to run Docker without `sudo`.

## Docker

Build the docker image (use the `-r` option to update the underlying images):
```shell
./docker/build.bash [-r]
```

Run the container:
```shell
./docker/run.bash
```

### Docker Compose

As an alternative to `build.bash` and `run.bash`, you can use Docker Compose.

Build the image:
```shell
docker compose -f docker/compose.yaml build
```

Run the container:
```shell
docker compose -f docker/compose.yaml up
```

Run the container in detached mode and open an interactive shell:
```shell
docker compose -f docker/compose.yaml up -d
docker exec -it mulinex bash
```

Stop the container:
```shell
docker compose -f docker/compose.yaml down
```

## Usage

Connect to the Raspberry with
```shell
ssh mulsbc@100.100.100.3
```
The password is `123456`.

The Raspberry will print a message confirming if chrony is synchronized or not. If it is not synchronized, run on the **NUC** and **outside** of the **container**
```shell
sudo systemctl restart chrony
```
to synchronize it. You can check the synchronization status with
```shell
check_chrony_100
```

### Launch Files

Launch the lidar with
```shell
ros2 launch sllidar_ros2 view_sllidar_s2e_launch.py device_ip:=192.168.11.2
```

Activate the joystick with
```shell
ros2 launch omni_mulinex_joystic start_joystic_nodes.launch.py
```

Activate the state broadcaster with
```shell
ros2 control load_controller state_broadcaster --set-state active
```

## Acknowledgements

- Attila De Benedittis
- Lorenz1 Martignetti
- Jacopo **Tiffany** Cioni (for grammatical errors)