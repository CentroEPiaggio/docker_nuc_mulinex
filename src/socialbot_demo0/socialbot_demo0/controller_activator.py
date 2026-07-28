from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ControllerActivator(Node):
    """Wait for the custom OmniController service and activate its internal state."""

    def __init__(self) -> None:
        super().__init__("demo0_controller_activator")
        self.declare_parameter("service_name", "/omni_controller/activate_srv")
        self.declare_parameter("retry_period", 1.0)

        service_name = str(self.get_parameter("service_name").value)
        retry_period = float(self.get_parameter("retry_period").value)

        self._client = self.create_client(SetBool, service_name)
        self._request_in_flight = False
        self._activated = False
        self._timer = self.create_timer(retry_period, self._try_activate)
        self.get_logger().info(f"Waiting for {service_name}")

    def _try_activate(self) -> None:
        if self._activated or self._request_in_flight:
            return

        if not self._client.service_is_ready():
            self._client.wait_for_service(timeout_sec=0.0)
            return

        request = SetBool.Request()
        request.data = True
        self._request_in_flight = True
        future = self._client.call_async(request)
        future.add_done_callback(self._activation_finished)

    def _activation_finished(self, future) -> None:
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS future exceptions are runtime dependent.
            self.get_logger().warning(f"Controller activation call failed: {exc}")
            return

        if response.success:
            self._activated = True
            self._timer.cancel()
            self.get_logger().info(f"OmniController activated: {response.message}")
        else:
            self.get_logger().warning(f"OmniController refused activation: {response.message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerActivator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
