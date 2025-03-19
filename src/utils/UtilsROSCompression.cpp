#include "UtilsROSCompression.hpp"

#include "rosbag2_compression/sequential_compression_reader.hpp"

namespace Utils::ROS::Compression
{
rosbag2_storage::BagMetadata
getCompressedBagMetadata(const QString& bagDirectory)
{
    auto reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>();

    rosbag2_storage::StorageOptions storageOptions;
    storageOptions.uri = bagDirectory.toStdString();
    rosbag2_cpp::ConverterOptions converterOptions;
    reader->open(storageOptions, converterOptions);

    const auto metaData = reader->get_metadata();
    reader->close();

    return metaData;
}
}
