#include "catch_ros2/catch_ros2.hpp"

#include "UtilsGeneral.hpp"

#include <filesystem>
#include <fstream>

TEST_CASE("Utils General Testing", "[utils]") {
    SECTION("Get file extension test") {
        REQUIRE(Utils::General::getFileExtension("file.txt") == "txt");
        REQUIRE(Utils::General::getFileExtension("archive.tar.gz") == "gz");
        REQUIRE(Utils::General::getFileExtension("./relative/path.json") == "json");
        REQUIRE(Utils::General::getFileExtension("video.MKV") == "MKV");
        REQUIRE(Utils::General::getFileExtension("noextension") == "");
        REQUIRE(Utils::General::getFileExtension("trailing_dot.") == "");
    }
    SECTION("Get available drive space test") {
        REQUIRE(Utils::General::getAvailableDriveSpace(".") > 0.0f);

        REQUIRE_NOTHROW(Utils::General::getAvailableDriveSpace("/this/path/does/not/exist"));
        REQUIRE_NOTHROW(Utils::General::getAvailableDriveSpace(""));
    }

    SECTION("Create and clear dir test") {
        const std::string path = "create_and_clear_test_dir";

        SECTION("Create") {
            REQUIRE(!std::filesystem::exists(path));

            Utils::General::createAndClearDirectory(path);
            REQUIRE(std::filesystem::exists(path));
            REQUIRE(std::filesystem::is_empty(path));
        }
        SECTION("Clear") {
            std::filesystem::create_directories(path + "/sub_dir");

            std::ofstream out(path + "/sub_dir/file.txt");
            out.close();

            Utils::General::createAndClearDirectory(path);
            REQUIRE(std::filesystem::exists(path));
            REQUIRE(std::filesystem::is_empty(path));
        }

        std::filesystem::remove_all(path);
    }
}
