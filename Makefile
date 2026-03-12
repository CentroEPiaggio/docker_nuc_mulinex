COMPOSE = docker compose -f docker/compose.yaml

build:
	bash docker/build.bash

run:
	bash docker/run.bash

start:
	$(COMPOSE) up -d

attach:
	docker exec -it mulinex bash

stop:
	$(COMPOSE) down
