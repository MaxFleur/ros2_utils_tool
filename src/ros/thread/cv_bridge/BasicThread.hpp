#pragma once

#include <QThread>

// Basic thread class, overridden by custom classes
class BasicThread : public QThread {
    Q_OBJECT
public:
    explicit
    BasicThread(const QString& bagDirectory,
                const QString& topicName,
                QObject*       parent = nullptr);

signals:
    // Maximum instances used as 100 percent reference for a progress bar
    void
    calculatedMaximumInstances(int count);

    // OpenCV video writer or capture opening might fail
    void
    openingCVInstanceFailed();

    // Update progress bars
    void
    progressChanged(int iteration,
                    int progress);

    void
    finished();

protected:
    const QString m_bagDirectory;
    const QString m_topicName;
};
