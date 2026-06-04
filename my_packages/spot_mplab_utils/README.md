# spot_mplab_utils

*Disclaimer: This package is a work in progress... It means to contain helpers and functions for SPOT.*



## Launch files:

### SPOT launch

SPOTs launch is fairly complicated (see spot_drive for details) and it supports lots of parameters. Some of these params (such as launch_image_publishers) only work if used as a command line argument.

The our_launchers/spotty.launch.py means to solve this problem with some workarounds in the launch file.

You should always make a copy of config/our_spotty.yaml, modify its values for your use-case and start it as:

```
ros2 launch our_launchers spotty.launch.py config_file:=path/to/your/config/file
```

Because of the modifications, some params won't be overwritten by cli params which is not the standard way. Either use this launch file with only the config_file param, or use the original launch file from BDAIInstitute.