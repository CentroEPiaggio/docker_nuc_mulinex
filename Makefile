COMPOSE = docker compose -f docker/docker-compose.yaml

build:
	bash docker/build.bash

start:
	$(COMPOSE) up -d

attach:
	docker exec -it mulinex bash

stop:
	$(COMPOSE) down
