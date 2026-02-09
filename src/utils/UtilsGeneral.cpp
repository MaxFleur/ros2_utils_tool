#include "UtilsGeneral.hpp"

#include <QFileInfo>

#include <filesystem>

namespace Utils::General
{
float
getAvailableDriveSpace(const QString& path)
{
    std::error_code errorCode;
    const auto spaceInfo = std::filesystem::space(path.toStdString(), errorCode);

    return static_cast<float>(spaceInfo.available) / static_cast<float>(Utils::General::GIGABYTE_IN_BYTES);
}


const QString
getFileExtension(const QString& path)
{
    QFileInfo fileInfo(path);
    return fileInfo.suffix();
}
}
