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


void
createAndClearDirectory(const std::string& directory)
{
    if (!std::filesystem::exists(directory)) {
        std::filesystem::create_directory(directory);
    }
    if (!std::filesystem::is_empty(directory)) {
        for (const auto& entry : std::filesystem::directory_iterator(directory)) {
            std::filesystem::remove_all(entry.path());
        }
    }
}
}
