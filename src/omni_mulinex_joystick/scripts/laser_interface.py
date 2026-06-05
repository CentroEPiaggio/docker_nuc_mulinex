#!/usr/bin/env python3
import asyncio
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from bleak import BleakClient, BleakScanner

SERVICE_UUID        = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
CHARACTERISTIC_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
DEVICE_NAME         = "arm_laser"


class LaserTriggerNode(Node):
    def __init__(self, ble_client, loop):
        super().__init__('laser_interface')
        self.ble_client = ble_client
        self.loop = loop
        self.sub = self.create_subscription(
            Empty, 'shoot', self.shoot_callback, 10)
        self.get_logger().info('BLE connected, ready')

    def shoot_callback(self, msg):
        future = asyncio.run_coroutine_threadsafe(self.send_trigger(), self.loop)
        future.result()  # wait for completion

    async def send_trigger(self):
        await self.ble_client.write_gatt_char(
            CHARACTERISTIC_UUID, bytes([0x01]), response=False)
        self.get_logger().info('Trigger sent')


async def main_async():
    print("Scanning for arm_laser...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)
    if device is None:
        print("Device not found")
        return

    async with BleakClient(device) as client:
        print(f"Connected to {device.name}")

        loop = asyncio.get_event_loop()
        rclpy.init()
        node = LaserTriggerNode(client, loop)

        # spin rclpy in a separate thread so asyncio loop stays free
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        # keep asyncio loop alive until ROS shuts down
        while rclpy.ok():
            await asyncio.sleep(0.1)

        node.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
