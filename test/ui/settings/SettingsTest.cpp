#include "catch_ros2/catch_ros2.hpp"

#include "AdvancedSettings.hpp"
#include "BagToImagesSettings.hpp"
#include "BagToVideoSettings.hpp"
#include "BasicSettings.hpp"
#include "DialogSettings.hpp"
#include "DummyBagSettings.hpp"
#include "EditBagSettings.hpp"
#include "MergeBagsSettings.hpp"
#include "Parameters.hpp"
#include "PublishSettings.hpp"
#include "VideoToBagSettings.hpp"

#include <QSettings>

TEST_CASE("Settings Testing", "[ui]") {
    QSettings qSettings;
    qSettings.beginGroup("dialog");
    qSettings.setValue("save_parameters", true);
    qSettings.endGroup();

    SECTION("Concept Test") {
        REQUIRE(SettingsParameter<int>);
        REQUIRE(SettingsParameter<bool>);
        REQUIRE(SettingsParameter<QString>);

        REQUIRE(!SettingsParameter<float>);
        REQUIRE(!SettingsParameter<double>);
        REQUIRE(!SettingsParameter<char>);
        REQUIRE(!SettingsParameter<std::string>);
    }
    SECTION("Input Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("basic");
            REQUIRE(!qSettings.value("source_dir").isValid());
            REQUIRE(!qSettings.value("topic_name").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::BasicParameters parameters;
            BasicSettings settings(parameters, "basic");

            parameters.sourceDirectory = "/source/dir";
            parameters.topicName = "/test_topic_name";
            settings.write();

            qSettings.beginGroup("basic");
            REQUIRE(qSettings.value("source_dir").isValid());
            REQUIRE(qSettings.value("topic_name").isValid());
            REQUIRE(qSettings.value("source_dir").toString() == "/source/dir");
            REQUIRE(qSettings.value("topic_name").toString() == "/test_topic_name");
            qSettings.endGroup();
        }
    }
    SECTION("Advanced Input Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("advanced");
            REQUIRE(!qSettings.value("target_dir").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::AdvancedParameters parameters;
            AdvancedSettings settings(parameters, "advanced");

            parameters.targetDirectory = "/target/dir";
            settings.write();

            qSettings.beginGroup("advanced");
            REQUIRE(qSettings.value("target_dir").isValid());
            REQUIRE(qSettings.value("target_dir").toString() == "/target/dir");
            qSettings.endGroup();
        }
    }
    SECTION("Bag to Images Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("images");
            REQUIRE(!qSettings.value("format").isValid());
            REQUIRE(!qSettings.value("quality").isValid());
            REQUIRE(!qSettings.value("switch_red_blue").isValid());
            REQUIRE(!qSettings.value("bw_images").isValid());
            REQUIRE(!qSettings.value("jpg_optimize").isValid());
            REQUIRE(!qSettings.value("png_bilevel").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::BagToImagesParameters parameters;
            BagToImagesSettings settings(parameters, "images");

            parameters.format = "jpg";
            parameters.quality = 10;
            parameters.exchangeRedBlueValues = true;
            parameters.useBWImages = true;
            parameters.jpgOptimize = true;
            parameters.pngBilevel = true;
            settings.write();

            qSettings.beginGroup("images");
            REQUIRE(qSettings.value("format").isValid());
            REQUIRE(qSettings.value("format").toString() == "jpg");
            REQUIRE(qSettings.value("quality").isValid());
            REQUIRE(qSettings.value("quality").toInt() == 10);
            REQUIRE(qSettings.value("switch_red_blue").isValid());
            REQUIRE(qSettings.value("switch_red_blue").toBool() == true);
            REQUIRE(qSettings.value("bw_images").isValid());
            REQUIRE(qSettings.value("bw_images").toBool() == true);
            REQUIRE(qSettings.value("jpg_optimize").isValid());
            REQUIRE(qSettings.value("jpg_optimize").toBool() == true);
            REQUIRE(qSettings.value("png_bilevel").isValid());
            REQUIRE(qSettings.value("png_bilevel").toBool() == true);
            qSettings.endGroup();
        }
    }
    SECTION("Bag to Video Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("video");
            REQUIRE(!qSettings.value("format").isValid());
            REQUIRE(!qSettings.value("fps").isValid());
            REQUIRE(!qSettings.value("hw_acc").isValid());
            REQUIRE(!qSettings.value("switch_red_blue").isValid());
            REQUIRE(!qSettings.value("bw_images").isValid());
            REQUIRE(!qSettings.value("lossless_images").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::BagToVideoParameters parameters;
            BagToVideoSettings settings(parameters, "video");

            parameters.format = "mkv";
            parameters.fps = 20;
            parameters.useHardwareAcceleration = true;
            parameters.exchangeRedBlueValues = true;
            parameters.useBWImages = true;
            parameters.lossless = true;
            settings.write();

            qSettings.beginGroup("video");
            REQUIRE(qSettings.value("format").isValid());
            REQUIRE(qSettings.value("format").toString() == "mkv");
            REQUIRE(qSettings.value("fps").isValid());
            REQUIRE(qSettings.value("fps").toInt() == 20);
            REQUIRE(qSettings.value("hw_acc").isValid());
            REQUIRE(qSettings.value("hw_acc").toBool() == true);
            REQUIRE(qSettings.value("switch_red_blue").isValid());
            REQUIRE(qSettings.value("switch_red_blue").toBool() == true);
            REQUIRE(qSettings.value("bw_images").isValid());
            REQUIRE(qSettings.value("bw_images").toBool() == true);
            REQUIRE(qSettings.value("lossless_images").isValid());
            REQUIRE(qSettings.value("lossless_images").toBool() == true);
            qSettings.endGroup();
        }
    }
    SECTION("Video to Bag Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("bag");
            REQUIRE(!qSettings.value("fps").isValid());
            REQUIRE(!qSettings.value("custom_fps").isValid());
            REQUIRE(!qSettings.value("hw_acc").isValid());
            REQUIRE(!qSettings.value("switch_red_blue").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::VideoToBagParameters parameters;
            VideoToBagSettings settings(parameters, "bag");

            parameters.fps = 40;
            parameters.useCustomFPS = true;
            parameters.useHardwareAcceleration = true;
            parameters.exchangeRedBlueValues = true;
            settings.write();

            qSettings.beginGroup("bag");
            REQUIRE(qSettings.value("fps").isValid());
            REQUIRE(qSettings.value("fps").toInt() == 40);
            REQUIRE(qSettings.value("custom_fps").isValid());
            REQUIRE(qSettings.value("custom_fps").toBool() == true);
            REQUIRE(qSettings.value("hw_acc").isValid());
            REQUIRE(qSettings.value("hw_acc").toBool() == true);
            REQUIRE(qSettings.value("switch_red_blue").isValid());
            REQUIRE(qSettings.value("switch_red_blue").toBool() == true);
            qSettings.endGroup();
        }
    }
    SECTION("Dummmy Bag Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("dummy");
            REQUIRE(!qSettings.value("topics").isValid());
            REQUIRE(!qSettings.value("msg_count").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::DummyBagParameters parameters;
            DummyBagSettings settings(parameters, "dummy");

            parameters.messageCount = 250;
            parameters.topics.push_back({ "string", "example_name" });
            settings.write();

            qSettings.beginGroup("dummy");
            REQUIRE(qSettings.value("msg_count").isValid());
            REQUIRE(qSettings.value("msg_count").toInt() == 250);

            const auto size = qSettings.beginReadArray("topics");
            for (auto i = 0; i < size; ++i) {
                qSettings.setArrayIndex(i);
                REQUIRE(qSettings.value("type").isValid());
                REQUIRE(qSettings.value("type").toString() == "string");
                REQUIRE(qSettings.value("name").isValid());
                REQUIRE(qSettings.value("name").toString() == "example_name");
            }
            REQUIRE(size == 1);
            qSettings.endArray();

            qSettings.endGroup();
        }
    }
    SECTION("Merge Bags Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("merge");
            REQUIRE(!qSettings.value("topics").isValid());
            REQUIRE(!qSettings.value("second_source").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::MergeBagsParameters parameters;
            MergeBagsSettings settings(parameters, "merge");

            parameters.secondSourceDirectory = "/path/to/other/bag";
            parameters.topics.push_back({ "topic", "/path/to/other/bag", true });
            settings.write();

            qSettings.beginGroup("merge");
            REQUIRE(qSettings.value("second_source").isValid());
            REQUIRE(qSettings.value("second_source").toString() == "/path/to/other/bag");

            const auto size = qSettings.beginReadArray("topics");
            for (auto i = 0; i < size; ++i) {
                qSettings.setArrayIndex(i);
                REQUIRE(qSettings.value("name").isValid());
                REQUIRE(qSettings.value("name").toString() == "topic");
                REQUIRE(qSettings.value("dir").isValid());
                REQUIRE(qSettings.value("dir").toString() == "/path/to/other/bag");
                REQUIRE(qSettings.value("is_selected").isValid());
                REQUIRE(qSettings.value("is_selected").toBool() == true);
            }
            REQUIRE(size == 1);
            qSettings.endArray();

            qSettings.endGroup();
        }
    }
    SECTION("Edit Bag Input Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("edit");
            REQUIRE(!qSettings.value("topics").isValid());
            REQUIRE(!qSettings.value("delete_source").isValid());
            REQUIRE(!qSettings.value("update_timestamps").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::EditBagParameters parameters;
            EditBagSettings settings(parameters, "edit");

            parameters.deleteSource = true;
            parameters.updateTimestamps = true;
            parameters.topics.push_back({ "renamed_topic", "original_topic", 42, 1337, true });
            settings.write();

            qSettings.beginGroup("edit");
            REQUIRE(qSettings.value("delete_source").isValid());
            REQUIRE(qSettings.value("delete_source").toBool() == true);
            REQUIRE(qSettings.value("update_timestamps").isValid());
            REQUIRE(qSettings.value("update_timestamps").toBool() == true);

            const auto size = qSettings.beginReadArray("topics");
            for (auto i = 0; i < size; ++i) {
                qSettings.setArrayIndex(i);
                REQUIRE(qSettings.value("renamed_name").isValid());
                REQUIRE(qSettings.value("renamed_name").toString() == "renamed_topic");
                REQUIRE(qSettings.value("original_name").isValid());
                REQUIRE(qSettings.value("original_name").toString() == "original_topic");
                REQUIRE(qSettings.value("lower_boundary").isValid());
                REQUIRE(qSettings.value("lower_boundary").toInt() == 42);
                REQUIRE(qSettings.value("upper_boundary").isValid());
                REQUIRE(qSettings.value("upper_boundary").toInt() == 1337);
                REQUIRE(qSettings.value("is_selected").isValid());
                REQUIRE(qSettings.value("is_selected").toBool() == true);
            }
            REQUIRE(size == 1);
            qSettings.endArray();

            qSettings.endGroup();
        }
    }
    SECTION("Publish Settings Test") {
        SECTION("Read") {
            qSettings.beginGroup("publish");
            REQUIRE(!qSettings.value("fps").isValid());
            REQUIRE(!qSettings.value("width").isValid());
            REQUIRE(!qSettings.value("height").isValid());
            REQUIRE(!qSettings.value("switch_red_blue").isValid());
            REQUIRE(!qSettings.value("loop").isValid());
            REQUIRE(!qSettings.value("hw_acc").isValid());
            REQUIRE(!qSettings.value("scale").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::PublishParameters parameters;
            PublishSettings settings(parameters, "publish");

            parameters.fps = 40;
            parameters.width = 1280;
            parameters.height = 720;
            parameters.exchangeRedBlueValues = true;
            parameters.loop = true;
            parameters.useHardwareAcceleration = true;
            parameters.scale = true;
            settings.write();

            qSettings.beginGroup("publish");
            REQUIRE(qSettings.value("fps").isValid());
            REQUIRE(qSettings.value("fps").toInt() == 40);
            REQUIRE(qSettings.value("width").isValid());
            REQUIRE(qSettings.value("width").toInt() == 1280);
            REQUIRE(qSettings.value("height").isValid());
            REQUIRE(qSettings.value("height").toInt() == 720);
            REQUIRE(qSettings.value("switch_red_blue").isValid());
            REQUIRE(qSettings.value("switch_red_blue").toBool() == true);
            REQUIRE(qSettings.value("loop").isValid());
            REQUIRE(qSettings.value("loop").toBool() == true);
            REQUIRE(qSettings.value("hw_acc").isValid());
            REQUIRE(qSettings.value("hw_acc").toBool() == true);
            REQUIRE(qSettings.value("scale").isValid());
            REQUIRE(qSettings.value("scale").toBool() == true);
            qSettings.endGroup();
        }
    }
    SECTION("Dialog Settings Test") {
        SECTION("Read") {
            qSettings.clear();
            qSettings.beginGroup("dialog");
            REQUIRE(!qSettings.value("save_parameters").isValid());
            REQUIRE(!qSettings.value("predefined_topic_names").isValid());
            REQUIRE(!qSettings.value("check_ros2_naming_convention").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::DialogParameters parameters;
            DialogSettings settings(parameters, "dialog");

            parameters.usePredefinedTopicNames = false;
            parameters.saveParameters = true;
            parameters.checkROS2NameConform = true;
            settings.write();

            qSettings.beginGroup("dialog");
            REQUIRE(qSettings.value("save_parameters").isValid());
            REQUIRE(qSettings.value("save_parameters").toBool() == true);
            REQUIRE(qSettings.value("predefined_topic_names").isValid());
            REQUIRE(qSettings.value("predefined_topic_names").toBool() == false);
            REQUIRE(qSettings.value("check_ros2_naming_convention").isValid());
            REQUIRE(qSettings.value("check_ros2_naming_convention").toBool() == true);
            qSettings.endGroup();
        }
    }

    qSettings.clear();
}
