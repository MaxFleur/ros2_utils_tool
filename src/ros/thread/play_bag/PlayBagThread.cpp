#include "PlayBagThread.hpp"

PlayBagThread::PlayBagThread(const Utils::UI::PlayBagParameters& playBagParameters,
                             QObject*                            parent) :
    QThread(parent), m_playBagParameters(playBagParameters)
{
}


void
PlayBagThread::run()
{
    rosbag2_storage::StorageOptions storageOptions;
    rosbag2_transport::PlayOptions playOptions;

    storageOptions.uri = m_playBagParameters.bagDirectory.toStdString();
    playOptions.loop = m_playBagParameters.loop;

    m_player = std::make_shared<rosbag2_transport::Player>(storageOptions, playOptions);
    m_player->play();
}
