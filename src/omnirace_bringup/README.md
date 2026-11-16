To run hw:

in mulsbc

```bash
ros2 launch pi3hat_hw_interface moteus_pi3hat_interface.launch.py [... ] omnirace.yaml

```


In NUC:

```bash
ros2 launch omnirace_bringup omnirace_bringup.launch.py record_bag:=true
```

The joystick will not be responsive in terms of twist but will respond to activate and deactivate the motors.


