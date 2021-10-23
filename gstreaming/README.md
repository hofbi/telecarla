# GStreaming

GStreamer ROS nodes for multimedia streaming

## Steup

1. Run `setup/install_gstreamer.sh` to install GStreamer or `setup/install_gstreamer_from_src.sh` to install from source
1. Now you should be able to build the project inside your catkin workspace with `catkin build`

## Run

```shell
# RTSP Server
roslaunch rtsp_server.launch

# RTSP Client
roslaunch rtsp_client host:=HOST_IP
```

Find all available configuration options in the above mention launch files.

### Additional Launch Files

```shell
# RTSP Server with video test source
roslaunch rtsp_server_videotestsrc.launch

# RTSP Server with video from v4l2 devices such as webcams
roslaunch rtsp_server_v4l2src.launch
```

## Development

```shell
# Fomrat the code
make format

# Check format
make check_format
```

### CLion

#### Code Format

1. Download and install [CLion](https://www.jetbrains.com/clion/)
1. Install FileWatchers plugin
1. Import file watchers config from [config/watchers_linux.xml](config/watchers_linux.xml)

#### ROS Integration

By default clion creates its own build folder. To use the catkin workspace build folder follow [this](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html#set-build-paths) guide.

#### Debugging

* Follow [this](https://www.jetbrains.com/help/clion/attaching-to-local-process.html) guide to attach the CLion debugger to running processes.
* Set `graph_viz` launch file argument to `true` to generate [GStreamer pipeline diagram](https://developer.ridgerun.com/wiki/index.php/How_to_generate_a_Gstreamer_pipeline_diagram_%28graph%29)
  * `.dot` file is generated in `ROS_HOME` (usually `~/.ros`)
  * Convert to png with `dot -Tpng pipeline.dot > pipeline.png`
* Set `GST_DEBUG` environment variable in server/client launch file to switch log level of GStreamer
  * [GStreamer Debugging Tools](https://gstreamer.freedesktop.org/documentation/tutorials/basic/debugging-tools.html?gi-language=c)
  * [GStreamer Debugging](https://developer.ridgerun.com/wiki/index.php?title=GStreamer_Debugging)

## Network

### Protocols & Ports

GStreamer RTSP Server uses UDP by default on port range [32768, 60999]. Config change can be done either on server or client side

```shell
// Server side
gst_rtsp_media_factory_set_protocols(factory, GST_RTSP_LOWER_TRANS_TCP);
gst_rtsp_media_factory_set_protocols(factory, GST_RTSP_LOWER_TRANS_UDP);

// Client side
// Add parameter to rtspsrc on pipeline definition
// Default: protocols=tcp+udp-mcast+udp
```
