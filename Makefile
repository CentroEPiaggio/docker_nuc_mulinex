COMPOSE_FILES ?= -f docker/compose.yaml
COMPOSE = docker compose $(COMPOSE_FILES)
DEMO0_EXEC = $(COMPOSE) exec mulinex bash -lc
DEMO0_SHELL = source /opt/ros/humble/setup.bash; source $$HOME/mulinex_ws/install/setup.bash

.PHONY: build run start attach stop \
	demo0-build demo0-validate demo0-shell demo0-audio demo0-sim \
	demo0-mapping demo0-teleop demo0-realsense \
	demo0-save-map demo0-navigation demo0-run demo0-run-no-audio \
	demo0-scripted demo0-check

build:
	bash docker/build.bash

run:
	bash docker/run.bash

start:
	$(COMPOSE) up -d

attach:
	$(COMPOSE) exec mulinex bash

stop:
	$(COMPOSE) down

demo0-build:
	$(DEMO0_EXEC) 'source /opt/ros/humble/setup.bash; cd $$HOME/mulinex_ws; python3 demo0/static_validate.py; colcon build --symlink-install --event-handlers console_direct+'

demo0-validate:
	$(DEMO0_EXEC) 'cd $$HOME/mulinex_ws; python3 demo0/static_validate.py'

demo0-shell:
	$(COMPOSE) exec mulinex bash

demo0-audio:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); $$HOME/mulinex_ws/demo0/test_audio.bash'

demo0-sim:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); ros2 launch socialbot_demo0 simulation.launch.py use_rviz:=true'

demo0-mapping:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); ros2 launch socialbot_demo0 mapping.launch.py'

demo0-teleop:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); ros2 run teleop_twist_keyboard teleop_twist_keyboard'

demo0-realsense:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); ros2 launch realsense2_camera rs_launch.py'

demo0-save-map:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); $$HOME/mulinex_ws/demo0/save_map.bash'

demo0-navigation:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); ros2 launch socialbot_demo0 navigation.launch.py'

demo0-run:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); ros2 launch socialbot_demo0 demo0.launch.py'

demo0-run-no-audio:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); ros2 launch socialbot_demo0 demo0.launch.py audio_enabled:=false'

demo0-scripted:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); SCRIPT=$$(ros2 pkg prefix --share socialbot_demo0)/config/interview_script_fully_scripted.yaml; ros2 launch socialbot_demo0 demo0.launch.py script_file:=$$SCRIPT'

demo0-check:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); $$HOME/mulinex_ws/demo0/check_topics.bash'
