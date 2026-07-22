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
    for (const auto& service : m_parameters.services) {
        if (!service.isSelected) {
            continue;
        }
        playOptions.services_to_filter.push_back(service.name.toStdString());
    }
    playOptions.rate = m_parameters.rate;
    // Play options are stored as nanoseconds, so we need to multiply accordingly
    playOptions.start_offset = m_parameters.offset * 1e9;
    playOptions.loop = m_parameters.loop;

    m_player = std::make_unique<rosbag2_transport::Player>(storageOptions, playOptions);
    m_player->play();
}


BagPlayer::~BagPlayer()
{
    m_player->stop();
}
