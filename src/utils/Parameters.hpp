#pragma once

#include <QString>
#include <QVector>

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
struct BagToImagesParameters : AdvancedParameters {
    QString format = "jpg";
    int     quality = 8;
    bool    exchangeRedBlueValues = false;
    bool    useBWImages = false;
    bool    jpgOptimize = false;
    bool    pngBilevel = false;
};
struct BagToVideoParameters : AdvancedParameters {
    QString format = "mp4";
    int     fps = 30;
    bool    useHardwareAcceleration = false;
    bool    exchangeRedBlueValues = false;
    bool    useBWImages = false;
    bool    lossless = false;
};
struct PCDsToBagParameters : AdvancedParameters {
    int rate = 5;
};
struct VideoToBagParameters : AdvancedParameters {
    int  fps = 30;
    bool useCustomFPS = false;
    bool useHardwareAcceleration = false;
    bool exchangeRedBlueValues = false;
};
struct EditBagParameters : AdvancedParameters {
    struct EditBagTopic {
        QString renamedTopicName = "";
        QString originalTopicName;
        size_t  lowerBoundary = 0;
        size_t  upperBoundary;
        bool    isSelected = true;
    };

    QVector<EditBagTopic> topics = {};
    bool                  deleteSource = false;
    bool                  updateTimestamps = false;
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

struct PublishParameters : AdvancedParameters {
    int  fps = 30;
    int  width = 1280;
    int  height = 720;
    bool exchangeRedBlueValues = false;
    bool loop = false;
    bool useHardwareAcceleration = false;
    bool scale = false;
};

struct DialogParameters {
    bool saveParameters = false;
    bool usePredefinedTopicNames = true;
    bool checkROS2NameConform = false;
};
}
