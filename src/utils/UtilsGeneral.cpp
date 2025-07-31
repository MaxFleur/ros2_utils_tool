#include "UtilsGeneral.hpp"

#include <filesystem>

namespace Utils::General
{
[[nodiscard]] float
getAvailableDriveSpace(const QString& path)
{
    std::error_code errorCode;
    const auto spaceInfo = std::filesystem::space(path.toStdString(), errorCode);

    return (float) spaceInfo.available / (float) Utils::General::GIGABYTE_IN_BYTES;
}
}
