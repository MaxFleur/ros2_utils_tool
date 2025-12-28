<div align="center">

   ![License badge](https://img.shields.io/badge/License-EUPLv1.2-blue.svg)
   ![C++ badge](https://img.shields.io/badge/C++-20-blue.svg)
   ![Tag badge](https://img.shields.io/badge/Release-v0.14.0-blue.svg)

</div>
<div align="center">

   ![Jazzy badge](https://github.com/MaxFleur/ros2_utils_tool/actions/workflows/jazzy.yml/badge.svg?event=push)
   ![Kilted badge](https://github.com/MaxFleur/ros2_utils_tool/actions/workflows/kilted.yml/badge.svg?event=push)
   ![Rolling badge](https://github.com/MaxFleur/ros2_utils_tool/actions/workflows/rolling.yml/badge.svg?event=push)

</div>

<p align="center">
  <img width="424" height="236" src="https://github.com/user-attachments/assets/9ee5cb03-35d4-4d94-a8aa-30438abe8c1f">
</p>

# Overview

The ros2_utils_tool package provides a complete UI-based tool for the everyday-usage of ROS2 with additional CLI support. Many tools have optional advanced options to customize outputs.

<p align="center">
  <img width="400" height="480" src="https://github.com/user-attachments/assets/027cf65c-b03e-4882-9f72-43761cd1db6e">
  <img width="360" height="480" src="https://github.com/user-attachments/assets/2847d943-9212-4672-9dd3-23e96e51b51e">
</p>
<p align="center">
  <img width="760" height="130" src="https://github.com/user-attachments/assets/b34534c8-c751-4fec-b58e-0643ea0b7f3f">
</p>

As of now, the tool provides the following functionalities:

| Tool  | Description | Additional CLI support |
|-------|-------------|:----------:|
| Bag to Video | Export a ROS bag video topic to a video |  X  |
| Video to Bag | Port a video file to a ROS bag |  X  |
| Bag to PCDs | Export a ROS bag point cloud topic to a set of pcd files  |  X  |
| PCDs to Bag | Port a set of PCD files to a ROS bag  |  X  |
| Bag to Images | Export a ROS bag video topic to an image sequence |  X  |
| Bag TF2 to File | Export a ROS bag tf2 topic to a json or yaml file |  X  |
| Edit Bag | Rename, remove or crop topics in a ROS bag |    |
| Merge Bags | Merge selected topics of two bags in a new ROS bag |  X  |
| UI-based Bag Recording | Record a ROS bag | (X) (`ros2 bag record`)  |
| Dummy Bag | Create a ROS bag with dummy data |  X  |
| Compress Bag | Compress a bag file |  X  |
| Decompress Bag | Decompress a bag file |  X  |
| Video as ROS Topic | Publish a video file as a ROS image_msg topic |  X  |
| Send TF2 | Send a static or non-static ROS transformation (tf2) |  X  |
| Image Sequence as ROS Topic | Publish a file with images as ROS image_msg Topic |  X  |
| UI-based topic/service information | UI-supported topic/service info vis | (X) (`ros2 topic list`/`ros2 service list`)  |
| UI-based Bag Info | UI-supported bag info vis | (X) (`ros2 bag info`) |

NOTE: The package is still under active development, so more tools might be added later in the future. Additionally, already existing features might expand and change constantly.



## Installation

### Dependencies

The following packages are required to be installed manually:
- [ROS2](https://docs.ros.org/en/jazzy/index.html). **Jazzy**, **kilted** and **rolling** are the major supported versions. For **humble**, there is still support, but no additional features will be released anymore (see the `humble` branch).
- [Qt6/Qt5](https://doc.qt.io/) for all UI as well as some convenience functionalities.
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

4. Build the project and source it:
```
colcon build
source install/setup.bash
```

## Usage

**Full UI**:
```
ros2 run ros2_utils_tool tool_ui
```

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

If you discover a new bug or wish for a new feature, feel free to open a new issue.\
If you want to contribute another feature, please use the provided Uncrustify file for code formatting. As the `main` branch is only updated for new versions or critical bugfixes, the `develop` branch is the most current one, providing the newest updates and features. So please open a merge request with `develop` as the target branch.
