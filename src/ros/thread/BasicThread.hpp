#pragma once

#include <QThread>

// Basic thread class, overridden by custom classes
class BasicThread : public QThread {
    Q_OBJECT
public:
    explicit
    BasicThread(const QString& sourceDirectory,
                const QString& topicName,
                QObject*       parent = nullptr);

signals:
    void
    informOfGatheringData();

    // Update progress displayal in widget
    void
    progressChanged(const QString& progressString,
                    int            progress);

    void
    finished();

    void
    processing();

    // Might fail in some cases (CV instance opening failed, invalid input params...)
    void
    failed();

protected:
    const std::string m_sourceDirectory;
    const std::string m_topicName;
};
