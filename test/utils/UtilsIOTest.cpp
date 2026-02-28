#include "catch_ros2/catch_ros2.hpp"

#include "Parameters.hpp"
#include "UtilsIO.hpp"

#include <QFile>

#include "yaml-cpp/yaml.h"

#include <filesystem>

TEST_CASE("Utils IO Testing", "[utils]") {
    SECTION("Read tf2 from file test") {
        Parameters::SendTF2Parameters parameters;
        const auto verify = [&parameters] (const QString& fileData, const QString& childFrameName, bool isJson, bool returnValue,
                                           double tX, double tY, double tZ, double rX, double rY, double rZ, double rW) {
            const auto fileName = isJson ? "file.json" : "file.yaml";

            QFile file(fileName);
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            file.write(fileData.toUtf8());
            file.close();

            REQUIRE((isJson ? Utils::IO::readTF2FromJson(fileName, parameters)
                            : Utils::IO::readTF2FromYAML(fileName, parameters)) == returnValue);

            REQUIRE(parameters.childFrameName == childFrameName);
            REQUIRE(parameters.translation[0] == tX);
            REQUIRE(parameters.translation[1] == tY);
            REQUIRE(parameters.translation[2] == tZ);
            REQUIRE(parameters.rotation[0] == rX);
            REQUIRE(parameters.rotation[1] == rY);
            REQUIRE(parameters.rotation[2] == rZ);
            REQUIRE(parameters.rotation[3] == rW);

            std::filesystem::remove_all(fileName);
        };

        SECTION("Json") {
            verify("{}", "", true, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            verify("{\"transforms\": [{\"child_frame_id\": \"test_id\",}]}", "", true, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            verify("{\"transforms\": [{\"child_frame_id\": \"test_id\",\"transform\": {\"translation\": {\"x\": 1.0,\"y\": 2.0,\"z\": 3.0}}}]}",
                   "", true, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            verify("{\"transforms\": {\"child_frame_id\": \"test_id\",\"transform\": {\"translation\": {\"x\": 1.0,\"y\": 2.0,},\"rotation\": {\"x\": 0.1,\"w\": 1.1}}}]}",
                   "", true, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            verify("{\"transforms\": [{\"child_frame_id\": \"test_id\",\"transform\": "
                   "{\"translation\": {\"x\": 1.0,\"y\": 2.0,\"z\": 3.0},\"rotation\": {\"x\": 0.1,\"y\": 0.2,\"z\": 0.3,\"w\": 1.1}}}]}",
                   "test_id", true, true, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 1.1);
        }
        SECTION("YAML") {
            verify("transforms:", "", false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

            YAML::Node node;

            node["transforms"]["child_frame_id"] = "test_id";
            verify(QString::fromStdString(YAML::Dump(node)), "", false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

            node["transforms"]["transform"]["translation"]["x"] = 1.0;
            node["transforms"]["transform"]["translation"]["y"] = 2.0;
            node["transforms"]["transform"]["translation"]["z"] = 3.0;
            verify(QString::fromStdString(YAML::Dump(node)), "", false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

            node["transforms"]["transform"]["rotation"]["x"] = 0.1;
            node["transforms"]["transform"]["rotation"]["w"] = 1.1;
            verify(QString::fromStdString(YAML::Dump(node)), "", false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

            node["transforms"]["transform"]["rotation"]["y"] = 0.2;
            node["transforms"]["transform"]["rotation"]["z"] = 0.3;
            verify(QString::fromStdString(YAML::Dump(node)), "test_id", false, true, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 1.1);
        }
    }

    SECTION("Write tf2 to file test") {
        Parameters::SendTF2Parameters parameters;
        parameters.childFrameName = "test_file_child";
        parameters.translation[0] = 0.1;
        parameters.translation[1] = 0.2;
        parameters.translation[2] = 0.3;
        parameters.rotation[0] = 1.0;
        parameters.rotation[1] = 2.0;
        parameters.rotation[2] = 3.0;
        parameters.rotation[3] = 1.5;

        const auto resetParameter = [] {
            Parameters::SendTF2Parameters parameters;
            for (size_t i = 0; i < parameters.translation.size(); ++i) {
                parameters.translation[i] = 0.0;
            }
            for (size_t i = 0; i < parameters.rotation.size(); ++i) {
                parameters.rotation[i] = i == 3 ? 1.0 : 0.0;
            }
            parameters.childFrameName = "";
        };

        const auto verifyParameter = [&parameters] {
            REQUIRE(parameters.childFrameName == "test_file_child");
            REQUIRE(parameters.translation[0] == 0.1);
            REQUIRE(parameters.translation[1] == 0.2);
            REQUIRE(parameters.translation[2] == 0.3);
            REQUIRE(parameters.rotation[0] == 1.0);
            REQUIRE(parameters.rotation[1] == 2.0);
            REQUIRE(parameters.rotation[2] == 3.0);
            REQUIRE(parameters.rotation[3] == 1.5);
        };

        const auto verify = [&parameters, resetParameter, verifyParameter] (const QString& fileName, bool isJson) {
            REQUIRE((isJson ? Utils::IO::writeTF2ToJson(fileName, parameters)
                            : Utils::IO::writeTF2ToYAML(fileName, parameters)) == true);
            resetParameter();

            REQUIRE((isJson ? Utils::IO::readTF2FromJson(fileName, parameters)
                            : Utils::IO::readTF2FromYAML(fileName, parameters)) == true);
            verifyParameter();

            std::filesystem::remove_all(fileName.toStdString());
        };

        SECTION("Json") {
            verify("file.json", true);
        }
        SECTION("YAML") {
            verify("file.yaml", false);
        }
    }
}
