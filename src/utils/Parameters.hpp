#pragma once

#include <QString>
#include <QVector>

#include <thread>

// Parameters used to store all kinds of information, for interaction
// between UI parts and saving input values for later reusage
namespace Parameters
{
// Related to storing bag topic info
struct BagContent {
    QString name = "";
};
struct SelectableBagContent : BagContent {
    bool isSelected = true;
};

// Main parameters for threads and input storage
struct BasicParameters {
    virtual
    ~BasicParameters() = default;

    QString sourceDirectory = "";
};
struct DummyBagParameters : BasicParameters {
    struct DummyBagTopic : BagContent {
        QString type;
    };

    QVector<DummyBagTopic> topics = {};
    int                    messageCount = 100;
    int                    rate = 10;
    bool                   useCustomRate = false;
};
struct SendTF2Parameters : BasicParameters {
    std::array<double, 3> translation = { 0.0, 0.0, 0.0 };
    std::array<double, 4> rotation = { 0.0, 0.0, 0.0, 1.0 };
    QString               childFrameName = "";
    int                   rate = 1;
    bool                  isStatic = true;
};

struct SelectableBagContentParameters : BasicParameters {
    QVector<SelectableBagContent> services = {};
    QVector<SelectableBagContent> topics = {};
};
struct PlayBagParameters : SelectableBagContentParameters {
    double rate = 1.0;
    double offset = 0.0;
    bool   loop = false;
    bool   publishServiceRequests = false;
};
struct RecordBagParameters : SelectableBagContentParameters {
    int  maxSizeInMB = 1024;
    int  maxDurationInSeconds = 60;

    bool showAdvancedOptions = false;
    bool includeHiddenTopics = false;
    bool includeUnpublishedTopics = false;
    bool useCustomSize = false;
    bool useCustomDuration = false;
    bool useCompression = false;
    bool isCompressionFile = false;
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
struct BagToYamlParameters : AdvancedParameters {
    bool writeSingleOutputFile = true;
};

struct DeleteSourceParameters : AdvancedParameters {
    bool deleteSource = false;
    bool compressPerMessage = false;
};
struct EditBagParameters : DeleteSourceParameters {
    struct EditBagTopic : SelectableBagContent {
        QString renamedName = "";
        size_t  lowerBoundary = 0;
        size_t  upperBoundary;
    };

    QVector<EditBagTopic> topics = {};
    bool                  updateTimestamps = false;
    bool                  compressTarget = false;
};
struct MergeBagsParameters : DeleteSourceParameters {
    struct MergeBagTopic : SelectableBagContent {
        // Topic names might be identical across bags, but cannot be identical in the same bag.
        // So store the bag directory as an additional identifier
        QString bagDir = "";
    };

    QVector<MergeBagTopic> topics = {};
    QString                secondSourceDirectory = "";
    bool                   compressTarget = false;
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
    bool useCompression = false;
    bool useCustomFPS = false;
    bool isCompressionJPEG = true;
};
struct PublishParameters : VideoParameters {
    int  width = 1280;
    int  height = 720;
    bool loop = false;
    bool scale = false;
};

// Parameters related to options dialog
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
