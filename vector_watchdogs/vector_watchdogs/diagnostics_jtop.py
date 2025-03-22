#!/usr/bin/env python
# -*- coding: utf-8 -*-

from amr_interfaces.srv import Fan
from amr_interfaces.srv import JetsonClocks
from amr_interfaces.srv import NVPModel
from amr_safety.jtop_utils import board_status
from amr_safety.jtop_utils import cpu_status
from amr_safety.jtop_utils import disk_status
from amr_safety.jtop_utils import emc_status
from amr_safety.jtop_utils import fan_status
from amr_safety.jtop_utils import gpu_status
from amr_safety.jtop_utils import other_status
from amr_safety.jtop_utils import power_status
from amr_safety.jtop_utils import ram_status
from amr_safety.jtop_utils import swap_status
from amr_safety.jtop_utils import temp_status
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
# from diagnostic_msgs.msg import KeyValue
import jtop
import rclpy
from rclpy.node import Node


class DiagnosticsJtop(Node):
    """Diagnostic Jtop Publisher."""

    def __init__(self):
        """Initialize the DiagnosticsJtop class."""
        super().__init__('diagnostics_jtop')

        # Publishers
        self.publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics/jetson', 10)

        # Services
        self.fan_srv = self.create_service(
            Fan, '/jtop/fan', self.fan_service)

        self.nvpmodel_srv = self.create_service(
            NVPModel, '/jtop/nvpmodel', self.nvpmodel_service)

        self.jetson_clocks_srv = self.create_service(
            JetsonClocks, '/jtop/jetson_clocks', self.jetson_clocks_service)

        # Declare Parameters
        self.declare_parameter('interval', 0.5)
        self.declare_parameter('level_error', 60)
        self.declare_parameter('level_warning', 40)
        self.declare_parameter('level_ok', 20)

        # Initialize Parameters
        self.level_options = {
            self.get_parameter('level_error')._value: DiagnosticStatus.ERROR,
            self.get_parameter('level_warning')._value: DiagnosticStatus.WARN,
            self.get_parameter('level_ok')._value: DiagnosticStatus.OK
        }

        timer_period = self.get_parameter('interval')._value
        self.timer = self.create_timer(timer_period, self.jetson_callback)

        self.i = 0
        self.jetson = jtop.jtop(interval=0.5)
        self.arr = DiagnosticArray()

        self.get_logger().info(
            'Jetson Stats has started with interval : {}\n'
            'You can run following:\n'
            '  1. $ros2 run rqt_topic rqt_topic \n'
            '  2. Services for controlling fan_speed, power_mode, jetson_clocks\n'
            .format(timer_period)
        )

    def start(self):
        """Start the jtop tool and initialize board status.

        This method starts the jtop monitoring tool, retrieves the board
        information, and sets the hardware name and board status for later use.
        """
        # Start the jtop
        self.jetson.start()

        # Extract the board information
        board = self.jetson.board

        # Define the hardware name
        self.hardware = board['platform']['Machine']
        self.board_status = board_status(self.hardware, board, 'board')

        # Set Callback
        # self.jetson.attach(self.jetson_callback)

    def fan_service(self, req, res):
        """Service callback to update the fan settings.

        Tries to set new fan mode and speed from the request. If setting the fan
        parameters fails, logs the error and returns the current fan settings.
        The method waits until the requested fan speed is applied before responding.

        Parameters:
            req: Request containing the desired fan mode and speed.
            res: Response to be populated with the new fan mode and speed.

        Returns:
            The updated response with the fan settings.
        """
        # Try to set new nvpmodel
        fan_mode = req.mode
        fan_speed = req.speed

        try:
            self.jetson.fan.profile = fan_mode
            self.jetson.fan.speed = fan_speed
        except jtop.JtopException as e:
            # Return same nvp model
            self.get_logger().error(f'Failed to set fan: {e}')
            fan_mode = str(self.jetson.fan.profile)
            fan_speed = self.jetson.fan.speed

        while self.jetson.ok():
            if self.jetson.fan.speed == fan_speed:
                break

        res.set_fan_mode = fan_mode
        res.set_fan_speed = fan_speed

        self.get_logger().info(
            'Incoming Request \n \
                  Fan Mode:{}\t Fan Speed:{};\n \
                    Current Status of Fan Mode{}\t Fan Speed:{}'.format(
                req.mode, req.speed, res.set_fan_mode, res.set_fan_speed
            )
        )

        return res

    def jetson_clocks_service(self, req, res):
        """Service callback to update the jetson clocks.

        Sets the Jetson clocks to the desired state given in the request and
        returns the updated status.

        Parameters:
            req: Request containing the desired jetson clocks status.
            res: Response that will contain the updated clocks status.

        Returns:
            The response with the new jetson clocks status.
        """
        # Set new jetson_clocks
        self.jetson.jetson_clocks = req.status

        res.done = req.status

        self.get_logger().info(
            'Incoming Request \n Jetson Clocks:{};\n \
                  Current Status of Jetson Clocks:{}'.format(req.status, res.done))

        return res

    def nvpmodel_service(self, req, res):
        """Service callback to update the nvpmodel.

        Attempts to set a new nvpmodel on the Jetson platform. If the request
        fails, the current nvpmodel is retained. The method waits until the new
        model is applied.

        Parameters:
            req: Request containing the desired nvpmodel.
            res: Response to be populated with the updated power mode.

        Returns:
            The response with the new power mode as a string.
        """
        # Try to set new nvpmodel
        cur_nvpmodel = self.jetson.nvpmodel
        self.get_logger().info('Incoming Request \n NVPModel:{};\n Current:{}'.format(
            req.nvpmodel, cur_nvpmodel))
        nvpmodel = req.nvpmodel

        try:
            self.jetson.nvpmodel = nvpmodel
        except jtop.JtopException:
            # Return same nvp model
            nvpmodel = self.jetson.nvpmodel

        while self.jetson.ok():
            if self.jetson.nvpmodel.id == nvpmodel:
                break

        res.power_mode = str(self.jetson.nvpmodel)

        return res

    def jetson_callback(self):
        """Timer callback to collect and publish Jetson diagnostics.

        This callback collects diagnostic data from various Jetson sensors such
        as CPU, GPU, memory, temperature, power, fan and disk status. It then
        populates a DiagnosticArray message with the collected statuses and
        publishes it.

        The callback is triggered at a fixed interval defined by the timer.
        """
        # Add Timestamp
        self.arr.header.stamp = self.get_clock().now().to_msg()

        # Status board and board info
        self.arr.status = [other_status(
            self.hardware, self.jetson, jtop.__version__)]

        # Make Diagnostic message for each CPU
        self.arr.status += [cpu_status(self.hardware, name, cpu)
                            for name, cpu in enumerate(self.jetson.cpu['cpu'])]

        # Make Diagnostic message for each GPU
        self.arr.status += [gpu_status(self.hardware, name, self.jetson.gpu[name])
                            for name in self.jetson.gpu]

        # Merge all other Diagnostics: RAM, SWAP, EMC
        self.arr.status += [ram_status(self.hardware,
                                       self.jetson.memory['RAM'], 'mem')]
        self.arr.status += [swap_status(self.hardware,
                                        self.jetson.memory['SWAP'], 'mem')]
        self.arr.status += [emc_status(self.hardware,
                                       self.jetson.memory['EMC'], 'mem')]

        # Temperature
        self.arr.status += [temp_status(self.hardware,
                                        self.jetson.temperature, self.level_options)]

        # Read Power
        self.arr.status += [power_status(self.hardware, self.jetson.power)]

        # Fan Controller
        if self.jetson.fan is not None:
            self.arr.status += [fan_status(self.hardware, key, value)
                                for key, value in self.jetson.fan.items()]

        # Status Board and board info
        self.arr.status += [self.board_status]

        # Add disk status
        self.arr.status += [disk_status(self.hardware,
                                        self.jetson.disk, 'board')]

        # Update status jtop
        self.publisher.publish(self.arr)


def main(args=None):
    """Start the diagnostics_jtop node."""
    rclpy.init(args=args)

    diagnostics_jtop = DiagnosticsJtop()
    diagnostics_jtop.start()

    rclpy.spin(diagnostics_jtop)
    diagnostics_jtop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
