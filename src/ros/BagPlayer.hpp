#pragma once

#include "Parameters.hpp"

#include "rosbag2_transport/player.hpp"

// Used to control a bag player
class BagPlayer {
public:
    explicit
    BagPlayer(const Parameters::PlayBagParameters& parameters);

    ~BagPlayer();

    void
    togglePlayerState(bool play)
    {
        play ? m_player->resume() : m_player->pause();
    }

    void
    setRate(double rate)
    {
        m_player->set_rate(rate);
    }

    void
    playNextMessage()
    {
        m_player->play_next();
    }

    QString
    getCurrentTimeAsString()
    {
        return QString::number(m_player->now().seconds(), 'f', 9);
    }

private:
    std::unique_ptr<rosbag2_transport::Player> m_player;

    const Parameters::PlayBagParameters& m_parameters;
};
