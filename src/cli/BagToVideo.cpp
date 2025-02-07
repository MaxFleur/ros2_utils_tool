#include "EncodingThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_video path/to/ROSBag path/of/stored/video\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Video topic inside the bag. If no topic name is specified, the first found video topic in the bag is taken.\n" << std::endl;
    std::cout << "-r or --rate: Framerate for the encoded video. Must be from 10 to 60." << std::endl;
    std::cout << "-a or --accelerate: Use hardware acceleration." << std::endl;
    std::cout << "-s or --switch: Switch red and blue values." << std::endl;
    std::cout << "-c or --colorless: Use colorless images." << std::endl;
    std::cout << "-l or --lossless (mkv only): Use lossless images.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);
    const auto arguments = app.arguments();
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Utils::UI::VideoInputParameters inputParameters;

    // Handle bag directory
    inputParameters.sourceDirectory = arguments.at(1);
    auto dirPath = inputParameters.sourceDirectory;
    if (!std::filesystem::exists(inputParameters.sourceDirectory.toStdString())) {
        std::cerr << "Bag file not found. Make sure that the bag file exists!" << std::endl;
        return 0;
    }
    if (const auto doesDirContainBag = Utils::ROS::doesDirectoryContainBagFile(inputParameters.sourceDirectory); !doesDirContainBag) {
        std::cerr << "The directory does not contain a bag file!" << std::endl;
        return 0;
    }

    // Video directory
    inputParameters.targetDirectory = arguments.at(2);
    dirPath = inputParameters.targetDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "The entered directory for the video file does not exist. Please specify a correct directory!" << std::endl;
        return 0;
    }
    inputParameters.format = inputParameters.targetDirectory.right(3);
    if (inputParameters.format != "mp4" && inputParameters.format != "mkv") {
        std::cerr << "The entered video name is not in correct format. Please make sure that the video file ends in mp4 or mkv!" << std::endl;
        return 0;
    }

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Topic name
        if (Utils::CLI::containsArguments(arguments, "-t", "--topic_name")) {
            const auto topicNameIndex = Utils::CLI::getArgumentsIndex(arguments, "-t", "--topic_name");
            if (arguments.at(topicNameIndex) == arguments.last()) {
                std::cerr << "Please enter the bag topic name!" << std::endl;
                return 0;
            }

            const auto& topicName = arguments.at(topicNameIndex + 1);
            if (!Utils::ROS::doesBagContainTopicName(inputParameters.sourceDirectory, topicName)) {
                std::cerr << "Topic has not been found in the bag file!" << std::endl;
                return 0;
            }
            if (Utils::ROS::getTopicType(inputParameters.sourceDirectory, topicName) != "sensor_msgs/msg/Image") {
                std::cerr << "The entered topic is not in sensor message format!" << std::endl;
                return 0;
            }
            inputParameters.topicName = topicName;
        }

        // Framerate
        if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", inputParameters.fps, 10, 60)) {
            std::cerr << "Please enter a framerate in the range of 10 to 60!" << std::endl;
            return 0;
        }

        // Hardware acceleration
        inputParameters.useHardwareAcceleration = Utils::CLI::containsArguments(arguments, "-a", "--accelerate");
        // Switch red and blue values
        inputParameters.switchRedBlueValues = Utils::CLI::containsArguments(arguments, "-s", "--switch");
        // Colorless
        inputParameters.useBWImages = Utils::CLI::containsArguments(arguments, "-c", "--colorless");
        // Lossless
        inputParameters.lossless = Utils::CLI::containsArguments(arguments, "-l", "--lossless");
    }

    // Search for topic name in bag file if not specified
    if (inputParameters.topicName.isEmpty()) {
        const auto& firstTopicWithImageType = Utils::ROS::getFirstTopicWithCertainType(inputParameters.sourceDirectory, "sensor_msgs/msg/Image");
        if (firstTopicWithImageType == std::nullopt) {
            std::cerr << "The bag file does not contain any image topics!" << std::endl;
            return 0;
        }

        inputParameters.topicName = *firstTopicWithImageType;
    }

    if (std::filesystem::exists(inputParameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The video already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create encoding thread and connect to its informations
    auto* const encodingThread = new EncodingThread(inputParameters);
    QObject::connect(encodingThread, &EncodingThread::openingCVInstanceFailed, [] {
        std::cerr << "The video writing failed. Please make sure that all inputParameters are set correctly and disable the hardware acceleration, if necessary." << std::endl;
        return 0;
    });
    QObject::connect(encodingThread, &EncodingThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(encodingThread, &EncodingThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Encoding finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(encodingThread, &EncodingThread::finished, encodingThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Encoding video. Please wait..." << std::endl;
    Utils::CLI::runThread(encodingThread, signalStatus);

    return EXIT_SUCCESS;
}
