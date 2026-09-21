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

void
createAndClearDirectory(const std::string& directory);

inline constexpr long GIGABYTE_IN_BYTES = 1073741824;
}
