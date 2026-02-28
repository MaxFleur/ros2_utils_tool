<img src="https://github.com/user-attachments/assets/9ee5cb03-35d4-4d94-a8aa-30438abe8c1f" height=110 align="right">

# ros2_utils_tool - An intuitive and powerful toolset for everyday ROS2 activities.

<p align="center">
  <img src="https://img.shields.io/badge/License-EUPLv1.2-blue.svg"/>
  <img src="https://img.shields.io/badge/C++-20-blue.svg"/>
  <img src="https://img.shields.io/badge/Release-v0.15.0-blue.svg"/>
  <img src="https://github.com/MaxFleur/ros2_utils_tool/actions/workflows/jazzy.yml/badge.svg?event=push"/>
  <img src="https://github.com/MaxFleur/ros2_utils_tool/actions/workflows/kilted.yml/badge.svg?event=push"/>
  <img src="https://github.com/MaxFleur/ros2_utils_tool/actions/workflows/rolling.yml/badge.svg?event=push"/>
</p>

<p align="center">
  <img width="400" height="480" src="https://github.com/user-attachments/assets/027cf65c-b03e-4882-9f72-43761cd1db6e">
  <img width="360" height="480" src="https://github.com/user-attachments/assets/2847d943-9212-4672-9dd3-23e96e51b51e">
</p>
<p align="center">
  <img width="760" src="https://github.com/user-attachments/assets/28f6f971-3d95-41bb-9483-eb06ea389909">
</p>

The _ros2_utils_tool_ package provides a large set of tools for various ROS2 activities. These include converting operations between bag and file data, UI tools for terminal based operations (for example playing and recording ROS bags) and publishing data from files.

- **🎨 UI support**: For _every_ tool, full UI support is provided. A comprehensive and intuitive design making use of overarching layout designs, prefilled UI elements and tooltips makes sure that you get everything done as fast as possible.
- **📟 Partial CLI support**: For almost any tool there is an official ROS2 or custom CLI option, providing fast and precise operations which can also be used for scripting. Plus, they give you the full hacker experience!
- **🚀 Highly optimized**: Many tools support multithreading or hardware acceleration, vastly speeding up operations. Additionally, you can configure the CPU usage to prevent your system from slowing down too much.
- **💾 Settings storage**: UI elements input can be stored for later reusage, allowing you to easily rerun any tool.
- **⚙️ Options**: _ros2_utils_tool_ provides additional functions for configuring warnings, input settings and more.

The most important information can be found here:
- [Provided tools](#provided-tools)
- [Installation](#installation)  
- [Usage](#usage)

## Provided tools

> [!Note]  
> This package is still under active development, so more tools might be added later in the future. Additionally, already existing tools might expand and change constantly.

<table>
  <thead>
    <tr>
      <th>Tool</th>
      <th>Description</th>
      <th>CLI support</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th colspan="3"></th>
    </tr>
    <tr>
      <th colspan="3" align="left">Conversion tools</th>
    </tr>
    <tr>
      <td><b>Bag to Video</b></td>
      <td>Export a ROS bag video topic to a video. Supports mp4, mkv and avi.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>Video to Bag</b></td>
      <td>Port a video file to a ROS bag. Supports mp4, mkv and avi.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>Bag to PCDs</b></td>
      <td>Export a ROS bag point cloud topic to a set of pcd files.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>PCDs to Bag</b></td>
      <td>Port a set of PCD files to a ROS bag.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>Bag to Images</b></td>
      <td>Export a ROS bag video topic to an image sequence. Images can be in jpg, png or bmp.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>Bag TF2 to File</b></td>
      <td>Export a ROS bag tf2 topic to a json or yaml file.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <th colspan="3"></th>
    </tr>
    <tr>
      <th colspan="3" align="left">Bag tools</th>
    </tr>
    <tr>
      <td><b>Edit Bag</b></td>
      <td>Create a bag file out of an existing one via renaming, removing or cropping topics.</td>
      <td align="center"></td>
    </tr>
    <tr>
      <td><b>Merge Bags</b></td>
      <td>Merge selected topics of two bags into a new bag file.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>Bag Recordings</b></td>
      <td>Record a ROS bag, UI based. Supports topic selection, compression, bag splitting and including hidden/unpublished topics.</td>
      <td align="center">X (<code>ros2 bag record</code>)</td>
    </tr>
    <tr>
      <td><b>Dummy Bag</b></td>
      <td>Create a ROS bag with dummy message data. Supported message types are images, strings, integers, point clouds and tf2.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>Compress Bag</b></td>
      <td>Compress a bag file.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>Decompress Bag</b></td>
      <td>Decompress a compressed bag file.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>Bag Playing</b></td>
      <td>Play a bag file, UI based. Supports topic selection and interactive commands such as stopping/resuming, rate adjustion and message skipping..</td>
      <td align="center">X (<code>ros2 bag play</code>)</td>
    </tr>
    <tr>
      <th colspan="3"></th>
    </tr>
    <tr>
      <th colspan="3" align="left">Publishing tools</th>
    </tr>
    <tr>
      <td><b>Video as ROS Topic</b></td>
      <td>Publish a video file to a ROS image message topic.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>Video as ROS Topic</b></td>
      <td>Publish a set of images to a ROS image message topic.</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <td><b>Send TF2</b></td>
      <td>Send a static or non-static ROS transformation (tf2).</td>
      <td align="center">X</td>
    </tr>
    <tr>
      <th colspan="3"></th>
    </tr>
    <tr>
      <th colspan="3" align="left">Info tools</th>
    </tr>
    <tr>
      <td><b>Topic/Service information</b></td>
      <td>UI based, show current topics and services with name and type, including publishers and subscribers.</td>
      <td align="center">X (<code>ros2 topic list</code>/<code>ros2 service list</code>)</td>
    </tr>
    <tr>
      <td><b>Bag Info</b></td>
      <td>UI based bag info vis.</td>
      <td align="center">X (<code>ros2 bag info</code>)</td>
    </tr>
  </tbody>
</table>

## Installation

### Prerequisites

A working ROS2 distribution is required. As of now, **jazzy**, **kilted** and **rolling** are supported. Make sure that your ROS distro is properly sourced:
```
source /opt/ros/distro_name/setup.bash
```

> [!Note]  
> As of version 0.14.0, **humble** is no longer supported and won't receive any more feature updates. However, critical bugfixes will still be included (see the `humble` branch).

### Dependencies

The following packages are required to be installed manually:
- [Qt6/Qt5](https://doc.qt.io/) for all UI features as well as some convenience functionalities.
- [catch2_ROS](https://index.ros.org/p/catch_ros2/) for Catch2-based unit tests with ROS2.

The following packages are also required, but should be installed automatically with ROS:
- [OpenCV](https://opencv.org/) for writing video files.
- [cv_bridge](https://index.ros.org/p/cv_bridge/) for converting ROS sensor images to cv matrices and vice versa.
- [PCL](https://pointclouds.org/) for creating point clouds and converting them from and to ROS messages.

The following packages are optional:
- [uncrustify](https://github.com/uncrustify/uncrustify) for code formatting.

All dependencies aside from ROS2 can be installed at once using the following command:

`sudo apt install libopencv-dev ros-&ROS_DISTRO-cv-bridge libpcl-dev qt6-base-dev qtbase5-dev ros-&ROS_DISTRO-catch-ros2`

For example for Jazzy:

`sudo apt install libopencv-dev ros-jazzy-cv-bridge libpcl-dev qt6-base-dev qtbase5-dev ros-jazzy-catch-ros2`

Alternatively, use `rosdep` to install all dependencies.

### Build the tool

1. Navigate to your ROS2 workspace's src direction:
```
cd path/to/your/workspace/src
```

2. Clone this repository:
```
git clone https://github.com/MaxFleur/ros2_utils_tool.git
```

3. Navigate back to your workspace:
```
cd path/to/your/workspace/
```

4. Build the project:
```
colcon build
```

## Usage

Always source your workspace first:
```
source install/setup.bash
```

### UI
```
ros2 run ros2_utils_tool tool_ui
```

### CLI tools

For each CLI tool, type `-h` or `--help` to get additional information regarding all flags, alongside example commands.

**Bag-to-Video-Tool**:
```
ros2 run ros2_utils_tool tool_bag_to_video /path/to/bag /path/to/video
```
(Note that a topic can be specified optionally. If no topic is specified, the first available video topic is used. The video needs to have an .mp4 or .mkv appendix).

**Video-to-Bag-Tool**:
```
ros2 run ros2_utils_tool tool_video_to_bag /path/to/video /path/to/bag
```
(Note that a topic can be specified optionally. If no topic is specified, a predefined topic name will be taken. The video needs to have an .mp4 or .mkv appendix).

**Bag-to-PCDs-Tool**:
```
ros2 run ros2_utils_tool tool_bag_to_pcds /path/to/bag /path/to/pcds
```
(Note that a topic can be specified optionally. If no topic is specified, the first available point cloud topic is used).

**PCDs-to-Bag-Tool**:
```
ros2 run ros2_utils_tool tool_bag_to_pcds /path/to/pcds /path/to/bag
```
(Note that a topic can be specified optionally. If no topic is specified, a predefined topic name will be taken).

**Bag-to-Images-Tool**:
```
ros2 run ros2_utils_tool tool_bag_to_images /path/to/bag /path/to/images
```
(Note that a topic can be specified optionally. If no topic is specified, the first available video topic is used. `image_format` needs to be either `jpg`, `bmp` or `png`, jpg is default).

**Bag-TF2-to-File-Tool**:
```
ros2 run ros2_utils_tool tool_tf2_to_file /path/to/bag /path/to/output
```
(Accepted file formats are json or yaml. Note that a topic can be specified optionally. If no topic is specified, the first available tf2 topic is used).

**Merge-Bags-Tool**:
```
ros2 run ros2_utils_tool tool_merge_bags path/to/first_bag path/to/second_bag -t1 topic_name_1 (...) -t2 topic_name_2 (...) path/to/output_bag
```

**Dummy-Bag-Tool**:
```
ros2 run ros2_utils_tool tool_dummy_bag path/to/bag topic_type_1 topic_name_1 ... message_count
```
(Topic type needs to be `String`, `Integer`, `Image` or `PointCloud`, up to four topics can be written, `message_count` needs to be a value from 1 to 1000).

**Compress-Bag-Tool**:
```
ros2 run mediassist4_ros_tools tool_compress_bag path/to/uncompressed/source/bag /path/to/compressed/target/bag
```
(Compression per file is default, use `-m message` to compress per message).

**Decompress-Bag-Tool**:
```
ros2 run mediassist4_ros_tools tool_decompress_bag path/to/compressed/source/bag /path/to/uncompressed/target/bag
```

**Publish-Video-Tool**:
```
ros2 run ros2_utils_tool tool_publish_video path/to/video
```
(Note that a topic can be specified optionally. If no topic is specified, a predefined topic name will be taken. The video needs to have an .mp4 or .mkv appendix).

**Publish-Images-Tool**:
```
ros2 run ros2_utils_tool tool_publish_images path/to/images
```
(Note that a topic can be specified optionally. If no topic is specified, a predefined topic name will be taken. Images need to be of format `jpg`, `bmp` or `png`).

**Send-TF2-Tool**:
```
ros2 run ros2_utils_tool tool_send_tf2
```
(Translation and rotation are specified using the flags `-t` and `-ro` respectively. Transformations are static per default, specifying an additional rate will send a nonstatic one instead).

**Unit tests**:
```
ros2 run ros2_utils_tool tool_tests
```

## License

The ros2_utils_tool package is licensed under [EUPLv1.2](https://interoperable-europe.ec.europa.eu/sites/default/files/custom-page/attachment/2020-03/EUPL-1.2%20EN.txt).

## Contribution 

If you discover a new bug or wish for a new feature, feel free to open a new issue.
> [!Note]  
> If you want to contribute another feature, please use the provided Uncrustify file for code formatting. As the `main` branch is only updated for new versions or critical bugfixes, the `develop` branch is the most current one, providing the newest updates and features. So please open a merge request with `develop` as the target branch.
