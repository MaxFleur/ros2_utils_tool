#pragma once

#include <QString>
#include <QVector>

#include <thread>

// Parameters used to configure the threads and
// store information in case an input widget is closed and reopened
namespace Parameters
{
struct BasicParameters {
    virtual
    ~BasicParameters() = default;

    QString sourceDirectory = "";
    QString topicName = "";
};
struct DummyBagParameters : BasicParameters {
    struct DummyBagTopic {
        QString type;
        QString name;
    };

    QVector<DummyBagTopic> topics = {};
    int                    messageCount = 100;
};

struct AdvancedParameters : BasicParameters {
    QString targetDirectory = "";
    bool    showAdvancedOptions = false;
};
struct MergeBagsParameters : AdvancedParameters {
    struct MergeBagTopic {
        QString name = "";
        // Topic names might be identical across bags, but cannot be identical in the same bag.
        // So store the bag directory as an additional identifier
        QString bagDir = "";
        bool    isSelected = true;
    };

    QVector<MergeBagTopic> topics = {};
    QString                secondSourceDirectory = "";
};
struct PCDsToBagParameters : AdvancedParameters {
    int rate = 5;
};

struct DeleteSourceParameters : AdvancedParameters {
    bool deleteSource = false;
};
struct EditBagParameters : DeleteSourceParameters {
    struct EditBagTopic {
        QString renamedTopicName = "";
        QString originalTopicName;
        size_t  lowerBoundary = 0;
        size_t  upperBoundary;
        bool    isSelected = true;
    };

    QVector<EditBagTopic> topics = {};
    bool                  updateTimestamps = false;
};
struct CompressBagParameters : DeleteSourceParameters {
    bool compressPerMessage = false;
};

struct RGBParameters : AdvancedParameters {
    bool exchangeRedBlueValues = false;
};
struct BagToImagesParameters : RGBParameters {
    QString format = "jpg";
    int     quality = 8;
    bool    useBWImages = false;
    bool    jpgOptimize = false;
    bool    pngBilevel = false;
};

struct VideoParameters : RGBParameters {
    int fps = 30;
};
struct BagToVideoParameters : VideoParameters {
    QString format = "mp4";
    bool    useBWImages = false;
    bool    lossless = false;
};
struct VideoToBagParameters : VideoParameters {
    bool useCustomFPS = false;
};
struct PublishParameters : VideoParameters {
    int  width = 1280;
    int  height = 720;
    bool loop = false;
    bool scale = false;
};

struct DialogParameters {
    int  maxNumberOfThreads = std::thread::hardware_concurrency();
    bool useHardwareAcceleration = false;
    bool saveParameters = false;
    bool usePredefinedTopicNames = true;
    bool checkROS2NameConform = false;
};
}
