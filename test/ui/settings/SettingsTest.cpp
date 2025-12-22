#include "catch_ros2/catch_ros2.hpp"

#include "AdvancedSettings.hpp"
#include "BagToImagesSettings.hpp"
#include "BagToVideoSettings.hpp"
#include "BasicSettings.hpp"
#include "DialogSettings.hpp"
#include "CompressBagSettings.hpp"
#include "DeleteSourceSettings.hpp"
#include "DummyBagSettings.hpp"
#include "EditBagSettings.hpp"
#include "MergeBagsSettings.hpp"
#include "Parameters.hpp"
#include "PCDsToBagSettings.hpp"
#include "PublishSettings.hpp"
#include "RecordBagSettings.hpp"
#include "RGBSettings.hpp"
#include "SendTF2Settings.hpp"
#include "TF2ToFileSettings.hpp"
#include "VideoSettings.hpp"
#include "VideoToBagSettings.hpp"

#include <QSettings>

void
checkSettingsInvalidacy(const QSettings& settings, const QVector<QString>& settingNames)
{
    for (const auto& settingName : settingNames) {
        REQUIRE(!settings.value(settingName).isValid());
    }
}


template<typename T>
concept SettingsReturnPrimitiveType = std::same_as<T, int> || std::same_as<T, bool> || std::same_as<T, double>;

// Verify modified settings
template<typename T>
requires SettingsReturnPrimitiveType<T>
void
verifiySettingPrimitive(const QSettings& settings, const QString& settingName, T expectedValue)
{
    REQUIRE(settings.value(settingName).isValid());

    if constexpr (std::is_same_v<T, int>) {
        REQUIRE(settings.value(settingName).toInt() == expectedValue);
    } else if constexpr (std::is_same_v<T, double>) {
        REQUIRE(settings.value(settingName).toDouble() == expectedValue);
    } else {
        REQUIRE(settings.value(settingName).toBool() == expectedValue);
    }
}


// Need this to allow qstring references
void
verifiySettingQString(const QSettings& settings, const QString& settingName, const QString& expectedValue)
{
    REQUIRE(settings.value(settingName).isValid());
    REQUIRE(settings.value(settingName).toString() == expectedValue);
}


TEST_CASE("Settings Testing", "[ui]") {
    QSettings qSettings;
    qSettings.beginGroup("dialog");
    qSettings.setValue("save_parameters", true);
    qSettings.endGroup();

    SECTION("Input Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("basic");
            checkSettingsInvalidacy(qSettings, { "source_dir", "topic_name" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::BasicParameters parameters;
            BasicSettings settings(parameters, "basic");

            parameters.sourceDirectory = "/source/dir";
            settings.write();

            qSettings.beginGroup("basic");
            verifiySettingQString(qSettings, "source_dir", "/source/dir");
            qSettings.endGroup();
        }
    }
    SECTION("Record Bag Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("record");
            checkSettingsInvalidacy(qSettings, { "topics", "all_topics", "show_advanced",
                                                 "include_hidden_topics", "include_unpublished_topics" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::RecordBagParameters parameters;
            RecordBagSettings settings(parameters, "record");

            parameters.topics.push_back({ "/topic" });
            parameters.allTopics = true;
            parameters.showAdvancedOptions = true;
            parameters.includeHiddenTopics = true;
            parameters.includeUnpublishedTopics = true;
            settings.write();

            qSettings.beginGroup("record");
            verifiySettingPrimitive(qSettings, "all_topics", true);
            verifiySettingPrimitive(qSettings, "show_advanced", true);
            verifiySettingPrimitive(qSettings, "include_hidden_topics", true);
            verifiySettingPrimitive(qSettings, "include_unpublished_topics", true);

            const auto size = qSettings.beginReadArray("topics");
            for (auto i = 0; i < size; ++i) {
                qSettings.setArrayIndex(i);
                verifiySettingQString(qSettings, "name", "/topic");
            }
            REQUIRE(size == 1);
            qSettings.endArray();

            qSettings.endGroup();
        }
    }
    SECTION("Dummmy Bag Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("dummy");
            checkSettingsInvalidacy(qSettings, { "topics", "msg_count", "rate", "use_custom_rate" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::DummyBagParameters parameters;
            DummyBagSettings settings(parameters, "dummy");

            parameters.messageCount = 250;
            parameters.rate = 25;
            parameters.useCustomRate = true;
            parameters.topics.push_back({ "string", "example_name" });
            settings.write();

            qSettings.beginGroup("dummy");
            verifiySettingPrimitive(qSettings, "msg_count", 250);
            verifiySettingPrimitive(qSettings, "rate", 25);
            verifiySettingPrimitive(qSettings, "use_custom_rate", true);

            const auto size = qSettings.beginReadArray("topics");
            for (auto i = 0; i < size; ++i) {
                qSettings.setArrayIndex(i);
                verifiySettingQString(qSettings, "type", "string");
                verifiySettingQString(qSettings, "name", "example_name");
            }
            REQUIRE(size == 1);
            qSettings.endArray();

            qSettings.endGroup();
        }
    }

    SECTION("Advanced Input Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("advanced");
            checkSettingsInvalidacy(qSettings, { "target_dir", "show_advanced" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::AdvancedParameters parameters;
            AdvancedSettings settings(parameters, "advanced");

            parameters.targetDirectory = "/target/dir";
            parameters.topicName = "/test_topic_name";
            parameters.showAdvancedOptions = true;
            settings.write();

            qSettings.beginGroup("advanced");
            verifiySettingQString(qSettings, "target_dir", "/target/dir");
            verifiySettingQString(qSettings, "topic_name", "/test_topic_name");
            verifiySettingPrimitive(qSettings, "show_advanced", true);
            qSettings.endGroup();
        }
    }
    SECTION("PCDs to Bag Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("pcds_to_bag");
            checkSettingsInvalidacy(qSettings, { "rate" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::PCDsToBagParameters parameters;
            PCDsToBagSettings settings(parameters, "pcds_to_bag");

            parameters.rate = 10;
            settings.write();

            qSettings.beginGroup("pcds_to_bag");
            verifiySettingPrimitive(qSettings, "rate", 10);
            qSettings.endGroup();
        }
    }
    SECTION("TF2 to File Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("tf2_to_file");
            checkSettingsInvalidacy(qSettings, { "keep_timestamps", "compact_output" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::TF2ToFileParameters parameters;
            TF2ToFileSettings settings(parameters, "tf2_to_file");

            parameters.compactOutput = true;
            parameters.keepTimestamps = true;
            settings.write();

            qSettings.beginGroup("tf2_to_file");
            verifiySettingPrimitive(qSettings, "keep_timestamps", true);
            verifiySettingPrimitive(qSettings, "compact_output", true);
            qSettings.endGroup();
        }
    }

    SECTION("Delete Source Input Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("delete_source_param");
            checkSettingsInvalidacy(qSettings, { "delete_source" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::EditBagParameters parameters;
            EditBagSettings settings(parameters, "delete_source_param");

            parameters.deleteSource = true;
            settings.write();

            qSettings.beginGroup("delete_source_param");
            verifiySettingPrimitive(qSettings, "delete_source", true);
            qSettings.endGroup();
        }
    }
    SECTION("Edit Bag Input Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("edit");
            checkSettingsInvalidacy(qSettings, { "topics", "update_timestamps" });
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
            verifiySettingPrimitive(qSettings, "update_timestamps", true);

            const auto size = qSettings.beginReadArray("topics");
            for (auto i = 0; i < size; ++i) {
                qSettings.setArrayIndex(i);
                verifiySettingQString(qSettings, "renamed_name", "renamed_topic");
                verifiySettingQString(qSettings, "original_name", "original_topic");
                verifiySettingPrimitive(qSettings, "lower_boundary", 42);
                verifiySettingPrimitive(qSettings, "upper_boundary", 1337);
                verifiySettingPrimitive(qSettings, "is_selected", true);
            }
            REQUIRE(size == 1);
            qSettings.endArray();

            qSettings.endGroup();
        }
    }
    SECTION("Merge Bags Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("merge");
            checkSettingsInvalidacy(qSettings, { "topics", "second_source" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::MergeBagsParameters parameters;
            MergeBagsSettings settings(parameters, "merge");

            parameters.secondSourceDirectory = "/path/to/other/bag";
            parameters.topics.push_back({ "topic", "/path/to/other/bag", true });
            settings.write();

            qSettings.beginGroup("merge");
            verifiySettingQString(qSettings, "second_source", "/path/to/other/bag");

            const auto size = qSettings.beginReadArray("topics");
            for (auto i = 0; i < size; ++i) {
                qSettings.setArrayIndex(i);
                verifiySettingQString(qSettings, "name", "topic");
                verifiySettingQString(qSettings, "dir", "/path/to/other/bag");
                verifiySettingPrimitive(qSettings, "is_selected", true);
            }
            REQUIRE(size == 1);
            qSettings.endArray();

            qSettings.endGroup();
        }
    }
    SECTION("Compress Bag Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("compress_bag");
            checkSettingsInvalidacy(qSettings, { "compress_per_message", "delete_source" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::CompressBagParameters parameters;
            CompressBagSettings settings(parameters, "compress_bag");

            parameters.compressPerMessage = true;
            parameters.deleteSource = true;
            settings.write();

            qSettings.beginGroup("compress_bag");
            verifiySettingPrimitive(qSettings, "compress_per_message", true);
            verifiySettingPrimitive(qSettings, "delete_source", true);
            qSettings.endGroup();
        }
    }

    SECTION("RGB Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("rgb");
            checkSettingsInvalidacy(qSettings, { "switch_red_blue" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::RGBParameters parameters;
            RGBSettings settings(parameters, "rgb");

            parameters.exchangeRedBlueValues = true;
            settings.write();

            qSettings.beginGroup("rgb");
            verifiySettingPrimitive(qSettings, "switch_red_blue", true);
            qSettings.endGroup();
        }
    }
    SECTION("Bag to Images Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("images");
            checkSettingsInvalidacy(qSettings, { "format", "quality", "bw_images", "jpg_optimize", "png_bilevel" });
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
            verifiySettingQString(qSettings, "format", "jpg");
            verifiySettingPrimitive(qSettings, "quality", 10);
            verifiySettingPrimitive(qSettings, "bw_images", true);
            verifiySettingPrimitive(qSettings, "jpg_optimize", true);
            verifiySettingPrimitive(qSettings, "png_bilevel", true);
            qSettings.endGroup();
        }
    }

    SECTION("Video Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("video");
            checkSettingsInvalidacy(qSettings, { "fps" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::VideoParameters parameters;
            VideoSettings settings(parameters, "video");

            parameters.fps = 15;
            settings.write();

            qSettings.beginGroup("video");
            verifiySettingPrimitive(qSettings, "fps", 15);
            qSettings.endGroup();
        }
    }
    SECTION("Bag to Video Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("video");
            checkSettingsInvalidacy(qSettings, { "format", "bw_images", "lossless_images" });
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
            verifiySettingQString(qSettings, "format", "mkv");
            verifiySettingPrimitive(qSettings, "bw_images", true);
            verifiySettingPrimitive(qSettings, "lossless_images", true);
            qSettings.endGroup();
        }
    }
    SECTION("Video to Bag Params Test") {
        SECTION("Read") {
            qSettings.beginGroup("bag");
            checkSettingsInvalidacy(qSettings, { "custom_fps" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::VideoToBagParameters parameters;
            VideoToBagSettings settings(parameters, "bag");

            parameters.useCustomFPS = true;
            settings.write();

            qSettings.beginGroup("bag");
            verifiySettingPrimitive(qSettings, "custom_fps", true);
            qSettings.endGroup();
        }
    }
    SECTION("Publish Settings Test") {
        SECTION("Read") {
            qSettings.beginGroup("publish");
            checkSettingsInvalidacy(qSettings, { "width", "height", "loop", "scale" });
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
            verifiySettingPrimitive(qSettings, "width", 1280);
            verifiySettingPrimitive(qSettings, "height", 720);
            verifiySettingPrimitive(qSettings, "loop", true);
            verifiySettingPrimitive(qSettings, "scale", true);
            qSettings.endGroup();
        }
    }

    SECTION("Send TF2 Test") {
        SECTION("Read") {
            qSettings.beginGroup("send_tf2");
            checkSettingsInvalidacy(qSettings, { "translation_x", "translation_y", "translation_z",
                                                 "rotation_x", "rotation_y", "rotation_z", "rotation_w",
                                                 "rate", "is_static" });
            qSettings.endGroup();
        }
        SECTION("Write") {
            Parameters::SendTF2Parameters parameters;
            SendTF2Settings settings(parameters, "send_tf2");

            parameters.translation[0] = 1.337;
            parameters.translation[1] = 4.2;
            parameters.translation[2] = 3.14159;
            parameters.rotation[0] = 1.234;
            parameters.rotation[1] = 6.7;
            parameters.rotation[2] = 2.71828;
            parameters.rotation[3] = -4.321;
            parameters.childFrameName = "test";
            parameters.rate = 3;
            parameters.isStatic = true;
            settings.write();

            qSettings.beginGroup("send_tf2");
            verifiySettingPrimitive(qSettings, "translation_x", 1.337);
            verifiySettingPrimitive(qSettings, "translation_y", 4.2);
            verifiySettingPrimitive(qSettings, "translation_z", 3.14159);
            verifiySettingPrimitive(qSettings, "rotation_x", 1.234);
            verifiySettingPrimitive(qSettings, "rotation_y", 6.7);
            verifiySettingPrimitive(qSettings, "rotation_z", 2.71828);
            verifiySettingPrimitive(qSettings, "rotation_w", -4.321);
            verifiySettingQString(qSettings, "name", "test");
            verifiySettingPrimitive(qSettings, "rate", 3);
            verifiySettingPrimitive(qSettings, "is_static", true);
            qSettings.endGroup();
        }
    }

    SECTION("Dialog Settings Test") {
        qSettings.clear();

        SECTION("Class values") {
            SECTION("Read") {
                qSettings.beginGroup("dialog");
                checkSettingsInvalidacy(qSettings, { "max_threads", "hw_acc", "save_parameters", "predefined_topic_names",
                                                     "warn_ros2_name_convention", "warn_target_overwrite", "warn_low_disk_space" });
                qSettings.endGroup();
            }
            SECTION("Write") {
                Parameters::DialogParameters parameters;
                DialogSettings settings(parameters, "dialog");

                parameters.maxNumberOfThreads = 4;
                parameters.useHardwareAcceleration = true;
                parameters.saveParameters = true;
                parameters.usePredefinedTopicNames = true;
                parameters.warnROS2NameConvention = true;
                parameters.warnTargetOverwrite = true;
                parameters.warnLowDiskSpace = true;
                settings.write();

                qSettings.beginGroup("dialog");
                verifiySettingPrimitive(qSettings, "max_threads", 4);
                verifiySettingPrimitive(qSettings, "hw_acc", true);
                verifiySettingPrimitive(qSettings, "save_parameters", true);
                verifiySettingPrimitive(qSettings, "predefined_topic_names", true);
                verifiySettingPrimitive(qSettings, "warn_ros2_name_convention", true);
                verifiySettingPrimitive(qSettings, "warn_target_overwrite", true);
                verifiySettingPrimitive(qSettings, "warn_low_disk_space", true);
                qSettings.endGroup();
            }
        }
        SECTION("Static") {
            SECTION("Read") {
                REQUIRE(DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()) == std::thread::hardware_concurrency());
                REQUIRE(DialogSettings::getStaticParameter("hw_acc", false) == false);
            }
            SECTION("Write") {
                DialogSettings::writeStaticParameter("max_threads", static_cast<unsigned int>(4));
                DialogSettings::writeStaticParameter("hw_acc", true);

                qSettings.beginGroup("dialog");
                verifiySettingPrimitive(qSettings, "max_threads", 4);
                verifiySettingPrimitive(qSettings, "hw_acc", true);
                qSettings.endGroup();
            }
        }
    }

    qSettings.clear();
}
