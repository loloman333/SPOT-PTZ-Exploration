# Our SPOT launchers
## 

- [ ] Fix debuggabble launches

---

This is a collection of ROS2 launchers that either wrap an existing one or launch it in a different manner.

Launches:
```bash
ros2 launch our_launchers spotty.launch.py
```
```bash
ros2 launch our_launchers ouster.launch.py
```
And the debug launches in the debug folder.

To see the launch arguments put `--show-args` prefix after the launch, like this:
```bash
ros2 launch our_launchers spotty.launch.py --show-args
```

In the `config` folder you can edit the launch parameters in `spotty_img_pub.yaml`  and the driver parameters in `our_spotty.yaml`.

### Configs:
There are three config yaml files in /config:
| File name | used in | desciption |
|-----------:|:------------|:------------|
| `our_spotty.yaml` | *spot_driver* | config file for the driver, like the robot's *ip*|
| `spotty_img_pub.yaml`  | *spot_driver* | launch arguments for the image publisher
| `ouster_config.yaml`  | *ouster_driver* | config file for the lidar

> [!TIP] 
> If you want your own config for your project, don't edit the ones here, create your own and parse the location of it as a ROS2 param! You can see the launchfiles params with `ros2 launch <pkg> <launch> --show-args` !


### Mock mode
```bash
ros2 launch our_launchers mock_spotty.launch.py
```
With this, the driver runs in **mock** mode, allowing you to start it without the robot—useful for testing. However, it's not a full virtual replica. Here's what it does and doesn’t mimic:  

**It mimics:** 
- The nodes responsible for *handling* the robot (but they don't really nothing by themselves.)
    - The service server (always returns `true`).  
    - The action server, however the simplest request crashes the driver (even with rosbag) so <u>dont attempt to use them in **mock**</u>.
    - Transform listener

**What it doesn’t:**  
- No topic publishing, so synchronous calls will hang unless you set a timeout (which is good practice anyway).  
    - (Exceptions: `spotty/status/feedback` and `../mobility_params` publish only zero or false values.  )
- No `odom` or `vision` frame, breaking any movement commands needing a reference.  


#### Is it useful?  
By itself not really. As does not publish any sensory info nor is there any real feedback in itself is not that useful.
It's use lies in pair with **ros2 bags**. Recording what you need for ample time, than playing back while the mock is running provides a great tool for testing and visualizing the environment. I recommend recording everything that is published, as you may not know what you need. With enough time this will create quite a large file, so watch out for that. I recommend [this](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) tutorial on how to use rosbags properly.

**some systax examples after `ros2 bag record`**

- To record everything : `-a`
- To record everything under the robot's namespace: `--regex "/spotty/.*"`
    - However this does not include the robot position or **tf**. Put the tf and tf_static topic as well, to fully record everything the robot does. : `$(ros2 topic list | grep "^/spotty/") /tf /tf_static` 
- And of course any plus topics you want to record, just list them in the command. (regex doesnt accept more topics)

With **mock** mode you can use the driver as a sort of listener, as when you can connect to the robot it **will** publish the topics. But the driver can't interact with robot in mock even when theres a connection, which can be a pro for some cases.

**Theres is bag recorder node in spot toolkit to make your life easier! I suggest checking it out!**