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
};
struct DummyBagParameters : BasicParameters {
    struct DummyBagTopic {
        QString type;
        QString name;
    };

    QVector<DummyBagTopic> topics = {};
    int                    messageCount = 100;
    int                    rate = 10;
    bool                   useCustomRate = false;
};
struct PlayBagParameters : BasicParameters {
    struct PlayBagTopic {
        QString name = "";
        QString type = "";
        bool    isSelected = true;
    };

    QVector<PlayBagTopic> topics = {};
    double                rate = 1.0;
    bool                  loop = false;
};
struct RecordBagParameters : BasicParameters {
    struct RecordBagTopic {
        QString name = "";
        bool    isSelected = true;
    };

    QVector<RecordBagTopic> topics = {};
    bool                    showAdvancedOptions = false;
    bool                    includeHiddenTopics = false;
    bool                    includeUnpublishedTopics = false;
};
struct SendTF2Parameters : BasicParameters {
    std::array<double, 3> translation = { 0.0, 0.0, 0.0 };
    std::array<double, 4> rotation = { 0.0, 0.0, 0.0, 1.0 };
    QString               childFrameName = "";
    int                   rate = 1;
    bool                  isStatic = true;
};

struct AdvancedParameters : BasicParameters {
    QString targetDirectory = "";
    QString topicName = "";
    bool    showAdvancedOptions = false;
};
struct PCDsToBagParameters : AdvancedParameters {
    int rate = 5;
};
struct TF2ToFileParameters : AdvancedParameters {
    bool keepTimestamps = false;
    bool compactOutput = true;
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
struct MergeBagsParameters : DeleteSourceParameters {
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
    bool useBWImages = false;
    bool lossless = false;
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
    unsigned int maxNumberOfThreads = std::thread::hardware_concurrency();
    bool         useHardwareAcceleration = false;
    bool         saveParameters = false;
    bool         usePredefinedTopicNames = true;
    bool         warnROS2NameConvention = false;
    bool         warnTargetOverwrite = true;
    bool         warnLowDiskSpace = true;
};
}
