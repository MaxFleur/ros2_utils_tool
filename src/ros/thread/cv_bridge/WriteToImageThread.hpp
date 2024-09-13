#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Encoding thread, encoding a video out of a ROSBag
class WriteToImageThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    WriteToImageThread(const Utils::UI::ImageParameters& imageParameters,
                       QObject*                          parent = nullptr);

    void
    run() override;

private:
    const QString m_imagesDirectory;
    const QString m_format;
    int m_quality;
};
