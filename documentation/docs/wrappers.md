# Wrappers for Simplified Development

The team at *Boston Dynamics* has developed numerous wrappers to streamline coding with ROS2. These abstractions make learning ROS code less critical, as many essential functions are already covered.

!!! info 
    This section is about the [ros_utilities](https://github.com/bdaiinstitute/ros_utilities/tree/1911832725d8a756a179a789b2e4c6be4d89a02e) package, *not* the [spot_wrapper](https://github.com/bdaiinstitute/spot_wrapper/tree/60a2c737b0f90fae76d9dcc49f6efc28e3a89a30) package. The spot_wrapper package is mainly for the bridge between the SDK and the ROS2 driver, so it is not really intended for the user to implement. Although for one section in it that we *may* have some use is the [calibration folder](https://github.com/bdaiinstitute/spot_wrapper/tree/60a2c737b0f90fae76d9dcc49f6efc28e3a89a30/spot_wrapper/calibration).

!!! warning
    Newer versions of the driver use the package name **synchros2** for this package, but older versions (from SDK version 4.1.0 and older) use the name **bdai_ros2_wrappers!** As this doc is for 4.1.0, the older name is used for the examples here, but note that boston's examples use the new one.

**First and foremost, read the official [documentation](https://github.com/bdaiinstitute/ros_utilities/tree/1911832725d8a756a179a789b2e4c6be4d89a02e/synchros2)! Some parts, like the process wide APIs are better explained there. Some are lacking, so I recommend a glance at this doc as well.** 

## Process wide APIs - Handling nodes with wrappers.

With some changes and extra features added by boston dynamics , the node can utilize the wrappers and features that the later sections explain in detail.
```python
import argparse
import logging
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.utilities import fqn, namespace_with

class MyNode():
    def __init__(self,robot_name: Optional[str] = None, node: Optional[Node] = None):

        #Boston goes around the ros logger, and uses the python logger
        logging.basicConfig()
        self._logger = logging.getLogger(fqn(self.__class__))
        self._logger.setLevel(logging.INFO) #Or debug

        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._robot_name = robot_name

        #setup basic utilities for the SPOT, subscribers, clients etc..
        #....

        self._logger.info("Setup done!")

    def helloworld(self)
        self._logger.info("Hello World!")

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    # add more arguments to your needs
    return parser

@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    """Command-line interface."""
    mynode = MyNode(args.robot, main.node)
    mynode.helloworld()
    return 0


if __name__ == "__main__":
    exit(main())
```
Here's an example of a ROS 2 node using the BDAI wrapper. As you can see, some differences exist compared to a standard node.

Instead of manually creating a node with `rclpy.create_node()`, the BDAI wrapper provides `ros_scope.node()`, which retrieves the current process node. The main function is structured differently, using `@ros_process.main(cli())` to handle initialization and argument parsing automatically. Logging is managed through Python’s `logging` module with `logging.getLogger(fqn(self.__class__))` instead of `self.get_logger()`. With these changes, we match boston dynamic's code structure, and avoid some repetitive tasks, such as starting, or stopping the node.

It's not necessary to make a node in a class, a simple function can be satisfactory for a simple task or test. To see how that's done look at `spot_examples/arm_simple` from the driver.


### 1. Subscribing to a Node
Instead of manually creating a node and a callback, you can leverage the subscription wrapper from **bdai_ros2_wrappers**. 

```python
from bdai_ros2_wrappers.subscription import wait_for_message
from spot_msgs.msg import PowerState

state = wait_for_message(PowerState, "/spotty/status/power_states", timeout = 5) 
```

The `wait_for_message` functions arguments:
- **topic_type**: The message type, e.g., `PowerState`.
- **topic_name**: The name of the topic, e.g., `/spotty/status/power_states`.
- **timeout**(optional): Timeout in seconds, returns `None` if reached.

These arguments and their explanation can also be found in function code. Almost all functions and abstractions have some sort of comment to help.

This method is efficient even for periodic data retrieval, matching the publisher's frequency.


***Tip:***  To ensure you import the correct data types as needed, use ROS tools like `rqt`'s topic monitor to identify topic types and names.

---

### 2. Services
Services operate on a client-server model, where multiple clients can request data or actions from a single server. Unlike the publisher-subscriber model, services allow for duplex communication (request-response). The `Serviced` class in **`bdai_ros2_wrappers.service`** simplifies creating service clients. 

#### Calling service
What do services do? Anything you write in the server's callback function. Most services in the spot driver are command services, simple command e.g. standing up. Let's look at an example:

From terminal (standard ROS2):
```shell
ros2 service call <service_name> <service_type> <optional_input_data>

#For example to stand:
ros2 service call /spotty/stand std_srvs/srv/Trigger
#The service type trigger needs no input data.
```
You can also do this in a more ergonomic way using rqt's service caller.

Python code:

```python
from bdai_ros2_wrappers.service import Serviced
from std_srvs.srv import Trigger

self.service = Serviced("/spotty/stand", Trigger, node)
#Node is optional but i recommend using it when inside a class. When not provided, the code creates a node, handles the service, than destroys that node.

request = Trigger.Request() 
response = Trigger.Response() #unnecesary, but now the IDE nows what datatype you work with.
response = self.service.synchronous(request)
#Request may be left out when dealing with empty requests (such as Trigger).
self._logger.info(response.success) #'true' or 'false' 
```
The Trigger service type is unique because data is not sent with the request, a specific service is just *triggered*. In other service types the request has to contain some data, that the service server can use. To state the obvious, messages classes vary, so for proper handling consult `rqt`'s service type browser.

## 3. ActionClientWrapper

A wrapper for the rclpy action_client is used to make use of the process wide API's, so that dealing with callbacks and other async and sync natured tasks are simplified. *At least thats what I think, Im still not an expert in the multi-threaded nature ros.*


```python
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from spot_msgs.action import RobotCommand #Other action types are available for the spot, but this one is used mainly.
from bdai_ros2_wrappers.utilities import namespace_with

#in some class..
self.action_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node)
#Create an action, as written in the Actions and services chapter...
action_goal = ...
timeout = 5.0 #in seconds, will wait forever if left None. 
result = self.action_client.sen_goal_and_wait("Robot is doing something", action_goal, timeout)
#here, process the the action based on the Action.Result
#or async:
handle = self._robot_command_client.send_goal_async_handle("Robot is doing something, but async", action_goal,
                                    result_callback=self.result_cb, feedback_callback=...)
timeout_bool = handle.wait_for_result(timeout)
```

The easiest method is to always send the action goal and wait for the result. Although its quick and easy, this reduces the control over the action. I recommend this for quick actions, like standing up, or moving the arm into a place. You can stack action goal into each other, resulting in very impressive simultaneous motions. To see how the actions are built, see the [Actions and Services](actions_and_services.md)

You can set a timeout to the synchronous calls, but be aware that if it times out, it will return a **None** , not an empty, or failed result.

Async calls work almost the same to the rclpy counterpart, with a few differences. First the function return the handle, not the future. Second the handle itself is wrapped, providing extra functionality. Without going too much into detail, see the [action client](https://github.com/bdaiinstitute/ros_utilities/blob/1911832725d8a756a179a789b2e4c6be4d89a02e/synchros2/synchros2/action_client.py) and the [action handle](https://github.com/bdaiinstitute/ros_utilities/blob/1911832725d8a756a179a789b2e4c6be4d89a02e/synchros2/synchros2/action_handle.py) file.

## 4. Tf listener wrapper.

***Because of it's importance in navigation, a separate [chapter](tf_and_navigation.md) is dedicated to the topic.***

## 5. Utilities

The first few function already came up in previous examples, I do recommend using them, they sometimes may not simplify, but make the code more elegant. Like `namespace_with`, it's possible to write every topic statically, but having a dynamic robot name, (or project name for your own topics) could be advantageous.
Heres the [code](https://github.com/bdaiinstitute/ros_utilities/blob/1911832725d8a756a179a789b2e4c6be4d89a02e/synchros2/synchros2/utilities.py) to take a look at.