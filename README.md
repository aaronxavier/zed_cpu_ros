# zed_cpu_ros

* simple ZED camera driver which only uses the CPU and doesn't need the official StereoLabs drivers / CUDA
* only publishes the left and right images and the camera info via ROS
* fork of Di Zeng's original version (https://github.com/willdzeng/zed_cpu_ros)
* additions of this fork:
  * independent high-frequency `std::thread` for frame grabbing (to prevent buffer lag)
  * `dynamic_reconfigure` support
  * better error and warning messages, e.g. when the frame rate drops
  * [Kalibr](https://github.com/ethz-asl/kalibr/) YAML file support
  * more launch / command line arguments
  * some C++11 constructs and code clean-up

## Usage
#### Install, build and source
* either use your existing catkin_ws:

    ```bash
    cd catkin_ws/src
    git clone https://github.com/MichaelGrupp/zed_cpu_ros
    cd ..
    catkin_make
    source devel/setup.bash
    ```
* for for quick & dirty testing create an in-source catkin_ws with `create_catkin_ws.sh`:

    ```bash
    git clone https://github.com/MichaelGrupp/zed_cpu_ros
    cd zed_cpu_ros
    . ./create_catkin_ws.sh --build
    ```

#### Get your calibration files: <br>
* either from the manufacturer:
    http://calib.stereolabs.com/?SN=XXXX

    Note: XXXX is your last four digit S/N of your camera, make sure to change it!!

* or use [Kalibr](https://github.com/ethz-asl/kalibr/) to calibrate yourself and use the "camchain" YAML file as the config file (requires the `config_is_kalibr_yaml` flag to be set in the launch file or as a command line argument)

* Put the .conf file into zed_cpu_ros/config folder and update the launch file configuration file name in zed_cpu_ros.launch to your SNXXXX.conf

    ```bash
    rosed zed_cpu_ros zed_cpu_ros.launch
    ```
    change XXXX to the .conf file you have, for example 1010 - in this line:
    ```xml
    <arg name="config_file_location" default="$(find zed_cpu_ros)/config/SN1010.conf"/>
    ```

#### Launch the ROS node

```bash
roslaunch zed_cpu_ros zed_cpu_ros.launch
```

You can reconfigure a few camera settings that are supported via the UVC interface of the ZED camera without restarting the node.

To open the reconfigure GUI:
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

### Launch file / command line parameters

Specify command line in the launch call as shown in this example:
```diff
roslaunch zed_cpu_ros zed_cpu_ros.launch frame_rate:=25 show_image:=true
```


 Parameter                    |           Description                                       |              Value          
------------------------------|-------------------------------------------------------------|-------------------------           
 `device`                   | UVC camera capture device ID                                | 0, 1, ...
 `resolution`               | ZED Camera resolution                                       | '0': HD2K                   
 _                            | _                                                           | '1': HD1080                 
 _                            | _                                                           | '2': HD720                  
 _                            | _                                                           | '3': VGA                                    
 `frame_rate`               | rate at which images are published                          | int                                        
 `config_file_location`     | path of ZED calibration file                                | string     
 `config_is_kalibr_yaml`    | calibration file is a "camchain" from [Kalibr](https://github.com/ethz-asl/kalibr/) | bool
 `show_image`               | display a window with image live view                       | bool
 `flip`                     | rotate the image by 180 degrees                             | bool              


## Authors:

* Di Zeng
* Michael Grupp
* and other contributors...
