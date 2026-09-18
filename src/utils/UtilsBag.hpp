#pragma once

#include "rosbag2_transport/record_options.hpp"

// Util functions for ROS bag processing
// Header-only to prevent globalized rosbag2_transport imports
namespace Utils::Bag
{
// Sets the compression settings of a record options instance
inline void
setCompressionOptions(rosbag2_transport::RecordOptions& recordOptions,
                      bool                              compressPerMessage,
                      int                               numberOfThreads)
{
    recordOptions.rmw_serialization_format = "cdr";
    recordOptions.compression_format = "zstd";
    recordOptions.compression_mode = compressPerMessage ? "message" : "file";
    recordOptions.compression_threads = numberOfThreads;
    // Need to set this to prevent message dropping
    recordOptions.compression_queue_size = 0;
}
}
