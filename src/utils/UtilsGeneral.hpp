#pragma once

#include <QString>

// Util functions for all sorts of things
namespace Utils::General
{
// Returns available drive space in gigabytes
[[nodiscard]] float
getAvailableDriveSpace(const QString& path);

// Get a file extension
[[nodiscard]] const QString
getFileExtension(const QString& path);

inline constexpr float MINIMUM_RECOMMENDED_DRIVE_SPACE = 10.737f;
inline constexpr long GIGABYTE_IN_BYTES = 1073741824;
}
