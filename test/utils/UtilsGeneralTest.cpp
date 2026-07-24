#include "catch_ros2/catch_ros2.hpp"

#include "UtilsGeneral.hpp"

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
}
