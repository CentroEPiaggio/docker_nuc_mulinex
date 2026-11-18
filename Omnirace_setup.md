This doc explain omnirace-specific changes made to this repository. 

### SSH access
SSH is available on port `2222` for the same user the container runs as. This enables code generation tools to access the container on NUC and to upload code. To access directly the container, you can use

```shell
ssh -p 2222 mulinex3@nuc-address
```


## How to run enverything

- NUC in AP mode
- Qualisys up and running, wifi connected to the NUC network

- ssh into nuc:
  - run `docker/run.bash` to start the container
    - run qualisys bridge 
    - ros2 run omni_bringup nuc_heartbeat_node --ros-args -p rate:=10.0
    - run omnirace manager?
  - ssh into sbc, run `check_chrony_100` to check if the time is synchronized, if not run `sudo systemctl restart chrony` on the NUC and check again until it is synchronized
    - run `ros2 launch blabla` to start the hardware interface and the joystick


On simulink: set host as mulinex3 @ 192.168.3.1:2222 password is mulinex. Workspace is /home/mulinex3/mulinex_ws or u can setup a new ws

