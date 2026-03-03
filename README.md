# Docker NUC Mulinex

## Preliminaries

Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine).
You can follow the installation method through `apt`.
Note that it makes you verify the installation by running `sudo docker run hello-world`.
It is better to avoid running this command with `sudo` and instead follow the post installation steps first and then run the command without `sudo`.

Follow with the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for Linux.
This will allow you to run Docker without `sudo`.

## Usage

Build the docker image (use the `-r` option to update the underlying images):
```shell
./docker/build.bash [-r]
```

Run the container:
```shell
./docker/run.bash
```

## Docker Compose

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
