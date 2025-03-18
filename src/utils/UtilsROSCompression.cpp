#include "UtilsROSCompression.hpp"

#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_cpp/reader.hpp"

#include <filesystem>

namespace Utils::ROS::Compression
{
bool
doesDirectoryContainBagFile(const QString& bagDirectory)
{
    auto sequentialCompressionReader = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(sequentialCompressionReader));

    try {
        reader->open(bagDirectory.toStdString());
    } catch (...) {
        return false;
    }

    const auto path = std::filesystem::path(bagDirectory.toStdString());
    std::filesystem::remove(path.string() + "/" + path.stem().string() + "_0.db3");
    reader->close();
    return true;
}


rosbag2_storage::BagMetadata
getBagMetadata(const QString& bagDirectory)
{
    auto sequentialCompressionReader = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(sequentialCompressionReader));

    reader->open(bagDirectory.toStdString());
    const auto metaData = reader->get_metadata();

    const auto path = std::filesystem::path(bagDirectory.toStdString());
    std::filesystem::remove(path.string() + "/" + path.stem().string() + "_0.db3");
    reader->close();
    return metaData;
}
}
