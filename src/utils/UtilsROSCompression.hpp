#pragma once

#include <QString>

#include "rosbag2_storage/bag_metadata.hpp"

// Compression related util functions
namespace Utils::ROS::Compression
{
// Returns the metadata stored for a ROS bag
[[nodiscard]] rosbag2_storage::BagMetadata
getCompressedBagMetadata(const QString& bagDirectory);
}
