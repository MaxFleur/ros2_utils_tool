#pragma once

#include "UtilsUI.hpp"

#include <QThread>

#include "rosbag2_transport/player.hpp"

// Thread used to play bag data
class PlayBagThread : public QThread {
    Q_OBJECT

public:
    explicit
    PlayBagThread(const Utils::UI::PlayBagParameters& playBagParameters,
                  QObject*                            parent = nullptr);

    void
    run() override;

#ifdef ROS_JAZZY
    /**
     * @note Due to API compatibility reasons (https://github.com/ros2/rosbag2/issues/1802),
     *       ROS Humble does not have a stop function for the player.
     *       So we can add this function only for Jazzy right now.
     */
    void
    stopPlaying()
    {
        m_player->stop();
    }

#else
    void
    pausePlaying()
    {
        m_player->pause();
    }

#endif

private:
    std::shared_ptr<rosbag2_transport::Player> m_player;

    const Utils::UI::PlayBagParameters& m_playBagParameters;
};
