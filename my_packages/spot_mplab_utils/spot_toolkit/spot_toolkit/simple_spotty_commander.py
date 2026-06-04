# This is an extended version of the Simple Spot Commander that allows you to send commands to a Spot robot via ROS 2 services. It includes a command-line interface for user interaction.

import argparse
import logging
from typing import Any, Dict, Optional, List

import synchros2.process as ros_process
import synchros2.scope as ros_scope
import rclpy
from synchros2.utilities import fqn, namespace_with
from rclpy.client import Client
from rclpy.node import Node
from std_srvs.srv import Trigger

TRIGGER_SERVICES = [
    "claim",
    "release",
    "stop",
    "self_right",
    "sit",
    "stand",
    "power_on",
    "power_off",
    "estop/hard",
    "estop/gentle",
    "estop/release",
]


class SimpleSpottyCommander:
    def __init__(self, robot_name: Optional[str] = "spotty", node: Optional[Node] = None, custom_srv_list: Optional[List[str]] = None, timeut_sec : float = None, raise_error = False) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._command_map: Dict[str, Client] = {}
        
        if custom_srv_list is None:
            custom_srv_list = TRIGGER_SERVICES
        
        for service_basename in custom_srv_list:
            service_name = namespace_with(robot_name, service_basename)
            self._command_map[service_basename] = node.create_client(Trigger, service_name)
            self._logger.info(f"Waiting for service {service_name}")
            if not self._command_map[service_basename].wait_for_service(timeut_sec):
                self._logger.error(f"Service {service_name} not available")
                if raise_error:
                    raise RuntimeError(f"Service {service_name} not available")
            else:
                self._logger.info(f"Found service {service_name}")

    def command(self, command: str) -> Any:
        try:
            return self._command_map[command].call(Trigger.Request())
        except KeyError:
            err = f"No command {command}"
            self._logger.error(err)
            return Trigger.Response(success=False, message=err)
        
    def command_async(self, command: str) -> Any:
        try:
            return self._command_map[command].async_send_request(Trigger.Request())
        except KeyError:
            err = f"No command {command}"
            self._logger.error(err)
            return Trigger.Response(success=False, message=err)

    def command_list(self) -> List[str]:
        """Returns a list of available commands."""
        return list(self._command_map.keys())

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", help="Name of the robot if the ROS driver is inside that namespace")
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    commander = SimpleSpottyCommander(args.robot)

    while rclpy.ok():
        cmd = input("Please enter a command:\n" + " ".join(TRIGGER_SERVICES) + "\n> ")
        result = commander.command(cmd)
        if not result.success:
            print("Error was", result.message)
        else:
            print("Successfully executed command")


if __name__ == "__main__":
    main()
