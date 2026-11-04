#pragma once

#include <QString>

#include <array>
#include <filesystem>

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

// Check whether a file path has one of the recognized image extensions
[[nodiscard]] bool
isImageFile(const std::filesystem::path& path);

inline constexpr std::array<std::string_view, 3> IMAGE_FILE_EXTENSIONS = { ".jpg", ".png", ".bmp" };

inline constexpr long GIGABYTE_IN_BYTES = 1073741824;
}
