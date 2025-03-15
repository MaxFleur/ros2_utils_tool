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
#include "PCDsToBagSettings.hpp"
#include "PublishSettings.hpp"
#include "RGBSettings.hpp"
#include "VideoSettings.hpp"
#include "VideoToBagSettings.hpp"

#include <QSettings>

TEST_CASE("Settings Testing", "[ui]") {
    QSettings qSettings;
    qSettings.beginGroup("dialog");
    qSettings.setValue("save_parameters", true);
    qSettings.endGroup();

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

    SECTION("Advanced Input Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("advanced");
            REQUIRE(!qSettings.value("target_dir").isValid());
            REQUIRE(!qSettings.value("show_advanced").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::AdvancedParameters parameters;
            AdvancedSettings settings(parameters, "advanced");

            parameters.targetDirectory = "/target/dir";
            parameters.showAdvancedOptions = true;
            settings.write();

            qSettings.beginGroup("advanced");
            REQUIRE(qSettings.value("target_dir").isValid());
            REQUIRE(qSettings.value("target_dir").toString() == "/target/dir");
            REQUIRE(qSettings.value("show_advanced").isValid());
            REQUIRE(qSettings.value("show_advanced").toBool() == true);
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
    SECTION("PCDs to Bag Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("pcds_to_bag");
            REQUIRE(!qSettings.value("rate").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::PCDsToBagParameters parameters;
            PCDsToBagSettings settings(parameters, "pcds_to_bag");

            parameters.rate = 10;
            settings.write();

            qSettings.beginGroup("pcds_to_bag");
            REQUIRE(qSettings.value("rate").isValid());
            REQUIRE(qSettings.value("rate").toInt() == 10);
            qSettings.endGroup();
        }
    }

    SECTION("RGB Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("rgb");
            REQUIRE(!qSettings.value("switch_red_blue").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::RGBParameters parameters;
            RGBSettings settings(parameters, "rgb");

            parameters.exchangeRedBlueValues = true;
            settings.write();

            qSettings.beginGroup("rgb");
            REQUIRE(qSettings.value("switch_red_blue").isValid());
            REQUIRE(qSettings.value("switch_red_blue").toBool() == true);
            qSettings.endGroup();
        }
    }
    SECTION("Bag to Images Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("images");
            REQUIRE(!qSettings.value("format").isValid());
            REQUIRE(!qSettings.value("quality").isValid());
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
            parameters.useBWImages = true;
            parameters.jpgOptimize = true;
            parameters.pngBilevel = true;
            settings.write();

            qSettings.beginGroup("images");
            REQUIRE(qSettings.value("format").isValid());
            REQUIRE(qSettings.value("format").toString() == "jpg");
            REQUIRE(qSettings.value("quality").isValid());
            REQUIRE(qSettings.value("quality").toInt() == 10);
            REQUIRE(qSettings.value("bw_images").isValid());
            REQUIRE(qSettings.value("bw_images").toBool() == true);
            REQUIRE(qSettings.value("jpg_optimize").isValid());
            REQUIRE(qSettings.value("jpg_optimize").toBool() == true);
            REQUIRE(qSettings.value("png_bilevel").isValid());
            REQUIRE(qSettings.value("png_bilevel").toBool() == true);
            qSettings.endGroup();
        }
    }

    SECTION("Video Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("video");
            REQUIRE(!qSettings.value("fps").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::VideoParameters parameters;
            VideoSettings settings(parameters, "video");

            parameters.fps = 15;
            settings.write();

            qSettings.beginGroup("video");
            REQUIRE(qSettings.value("fps").isValid());
            REQUIRE(qSettings.value("fps").toInt() == 15);
            qSettings.endGroup();
        }
    }
    SECTION("Bag to Video Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("video");
            REQUIRE(!qSettings.value("format").isValid());
            REQUIRE(!qSettings.value("bw_images").isValid());
            REQUIRE(!qSettings.value("lossless_images").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::BagToVideoParameters parameters;
            BagToVideoSettings settings(parameters, "video");

            parameters.format = "mkv";
            parameters.useBWImages = true;
            parameters.lossless = true;
            settings.write();

            qSettings.beginGroup("video");
            REQUIRE(qSettings.value("format").isValid());
            REQUIRE(qSettings.value("format").toString() == "mkv");
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
            REQUIRE(!qSettings.value("custom_fps").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::VideoToBagParameters parameters;
            VideoToBagSettings settings(parameters, "bag");

            parameters.useCustomFPS = true;
            settings.write();

            qSettings.beginGroup("bag");
            REQUIRE(qSettings.value("custom_fps").isValid());
            REQUIRE(qSettings.value("custom_fps").toBool() == true);
            qSettings.endGroup();
        }
    }
    SECTION("Publish Settings Test") {
        SECTION("Read") {
            qSettings.beginGroup("publish");
            REQUIRE(!qSettings.value("width").isValid());
            REQUIRE(!qSettings.value("height").isValid());
            REQUIRE(!qSettings.value("loop").isValid());
            REQUIRE(!qSettings.value("scale").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::PublishParameters parameters;
            PublishSettings settings(parameters, "publish");

            parameters.width = 1280;
            parameters.height = 720;
            parameters.loop = true;
            parameters.scale = true;
            settings.write();

            qSettings.beginGroup("publish");
            REQUIRE(qSettings.value("width").isValid());
            REQUIRE(qSettings.value("width").toInt() == 1280);
            REQUIRE(qSettings.value("height").isValid());
            REQUIRE(qSettings.value("height").toInt() == 720);
            REQUIRE(qSettings.value("loop").isValid());
            REQUIRE(qSettings.value("loop").toBool() == true);
            REQUIRE(qSettings.value("scale").isValid());
            REQUIRE(qSettings.value("scale").toBool() == true);
            qSettings.endGroup();
        }
    }

    SECTION("Dialog Settings Test") {
        qSettings.clear();

        SECTION("Read") {
            qSettings.beginGroup("dialog");
            REQUIRE(!qSettings.value("max_threads").isValid());
            REQUIRE(!qSettings.value("hw_acc").isValid());
            REQUIRE(!qSettings.value("save_parameters").isValid());
            REQUIRE(!qSettings.value("predefined_topic_names").isValid());
            REQUIRE(!qSettings.value("check_ros2_naming_convention").isValid());
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::DialogParameters parameters;
            DialogSettings settings(parameters, "dialog");

            parameters.maxNumberOfThreads = 4;
            parameters.useHardwareAcceleration = true;
            parameters.saveParameters = true;
            parameters.usePredefinedTopicNames = true;
            parameters.checkROS2NameConform = true;
            settings.write();

            qSettings.beginGroup("dialog");
            REQUIRE(qSettings.value("max_threads").isValid());
            REQUIRE(qSettings.value("max_threads").toInt() == 4);
            REQUIRE(qSettings.value("hw_acc").isValid());
            REQUIRE(qSettings.value("hw_acc").toBool() == true);
            REQUIRE(qSettings.value("save_parameters").isValid());
            REQUIRE(qSettings.value("save_parameters").toBool() == true);
            REQUIRE(qSettings.value("predefined_topic_names").isValid());
            REQUIRE(qSettings.value("predefined_topic_names").toBool() == true);
            REQUIRE(qSettings.value("check_ros2_naming_convention").isValid());
            REQUIRE(qSettings.value("check_ros2_naming_convention").toBool() == true);
            qSettings.endGroup();

            // Static functions
            REQUIRE(DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()) == 4);
            REQUIRE(DialogSettings::getStaticParameter("hw_acc", false) == true);
            REQUIRE(DialogSettings::getStaticParameter("save_parameters", false) == true);
        }
    }

    qSettings.clear();
}
