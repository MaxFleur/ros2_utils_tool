#include "BagPlayer.hpp"

BagPlayer::BagPlayer(const Parameters::PlayBagParameters& parameters) : m_parameters(parameters)
{
    rosbag2_storage::StorageOptions storageOptions;
    storageOptions.uri = m_parameters.sourceDirectory.toStdString();

    rosbag2_transport::PlayOptions playOptions;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }
        playOptions.topics_to_filter.push_back(topic.name.toStdString());
    }
    playOptions.rate = m_parameters.rate;
    playOptions.loop = m_parameters.loop;

    // Disable terminal logging
    auto result = rcutils_logging_set_logger_level("rosbag2_player", RCUTILS_LOG_SEVERITY_FATAL);
    Q_UNUSED(result);
    m_player = std::make_unique<rosbag2_transport::Player>(storageOptions, playOptions);
    m_player->play();
}


BagPlayer::~BagPlayer()
{
    m_player->stop();
}
