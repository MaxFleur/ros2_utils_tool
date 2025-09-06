#include "BagToFileThread.hpp"

#include "UtilsROS.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

#include <filesystem>

BagToFileThread::BagToFileThread(const Parameters::BagToFileParameters& parameters, QObject* parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent), m_parameters(parameters)
{
}


void
BagToFileThread::run()
{
    emit informOfGatheringData();

    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    const auto& bagMetaData = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);

    auto totalInstances = 0;
    for (const auto& topicInformation : bagMetaData.topics_with_message_count) {
        totalInstances += topicInformation.message_count;
    }


    RosMsgParser::ParsersCollection<RosMsgParser::ROS2_Deserializer> parser;
    std::set<std::pair<std::string, std::string> > bagTopicTypes;

    for (const auto& topicInformation : bagMetaData.topics_with_message_count) {
        bagTopicTypes.insert(std::make_pair(topicInformation.topic_metadata.name, topicInformation.topic_metadata.type));
    }
    for (const auto& bagTopicType : bagTopicTypes) {
        parser.registerParser(bagTopicType.first, RosMsgParser::ROSType(bagTopicType.second), RosMsgParser::GetMessageDefinition(bagTopicType.second));
    }

    auto reader = std::make_shared<rosbag2_cpp::Reader>();
    reader->open(m_sourceDirectory);

    while (reader->has_next()) {
        auto bagMessage = reader->read_next();

        const auto data = bagMessage->serialized_data->buffer;
        const auto length = bagMessage->serialized_data->buffer_length;
        std::vector<uint8_t> buffer(data, data + length);

        /*
         * auto flatContainer = parser.deserialize(bagMessage->topic_name, RosMsgParser::Span<uint8_t>(buffer));
         * for (auto& it : flatContainer->value) {
         *  std::cout << "Value:\n";
         *  std::cout << it.first << " >> " << it.second.convert<double>() << " " << parser.getParser("string") << std::endl;
         * }
         * for (auto& it : flatContainer->name) {
         *  std::cout << "Name:\n";
         *  std::cout << it.first << " >> " << it.second << " " << parser.getParser("string") << std::endl;
         * }*/
    }
}
