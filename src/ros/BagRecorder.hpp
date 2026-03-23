#pragma once

#include "Parameters.hpp"

#include <QObject>

#include "rosbag2_transport/recorder.hpp"

// Used to control a bag recorder from the UI
class BagRecorder : public QObject
{
    Q_OBJECT

public:
    explicit
    BagRecorder(const Parameters::RecordBagParameters& parameters);

    ~BagRecorder();

    void
    toggleState(bool play)
    {
        play ? m_recorder->resume() : m_recorder->pause();
    }

signals:
    void
    recorderStarted();

    void
    topicSubscribed(const QString& topic);

    void
    allTopicsSubscribed();

private:
    void
    searchForNewSubscription();

    void
    searchForAllSubscriptions();

private:
    std::shared_ptr<rosbag2_transport::Recorder> m_recorder;

    std::thread m_spinThread;

    std::future<void> m_topicSubscribedFuture;
    std::future<void> m_allTopicsSubscribedFuture;

    rclcpp::Rate m_rate { 50 };

    size_t m_selectedTopicsCount{ 0 };
    // Read and written across futures, so keep atomic
    std::atomic<bool> m_abortCalled{ false };

    const Parameters::RecordBagParameters& m_parameters;
};
