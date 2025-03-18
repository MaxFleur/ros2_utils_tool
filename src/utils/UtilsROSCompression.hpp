#pragma once

#include <QString>

#include "rosbag2_storage/bag_metadata.hpp"

// ROS compression related util functions
namespace Utils::ROS::Compression
{
// If a directory contains a valid compressed ROS bag
[[nodiscard]] bool
doesDirectoryContainBagFile(const QString& bagDirectory);

// Returns the metadata stored for a compressed ROS bag
[[nodiscard]] rosbag2_storage::BagMetadata
getBagMetadata(const QString& bagDirectory);
}
