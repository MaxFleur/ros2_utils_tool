#pragma once

#include <QPointer>
#include <QWidget>

class QLabel;

// Used to display a warning icon of low disk space along with the available disk space
class LowDiskSpaceWidget : public QWidget
{
    Q_OBJECT

public:
    LowDiskSpaceWidget(QWidget* parent = 0);

public:
    void
    setVisibility(const QString& path);

    [[nodiscard]] bool
    isDiskSpaceSpaceSufficient()
    {
        return m_isDiskSpaceSufficient;
    }

private:
    void
    setPixmapLabelIcon() const;

    bool
    event(QEvent *event);

private:
    QPointer<QLabel> m_warningIconLabel;
    QPointer<QLabel> m_diskSpaceLabel;

    bool m_isDiskSpaceSufficient;

    static constexpr int ICON_SIZE = 25;
};
