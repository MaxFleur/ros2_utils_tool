#include "WriteToBagThread.hpp"

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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_video_to_bag path/to/video path/of/stored/ros_bag\n" << std::endl;
    std::cout << "The video must have an ending of .mp4 or .mkv." << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Topic name. If this is empty, the name '/topic_video' will be taken.\n" << std::endl;
    std::cout << "-r or --rate: Framerate for the image stream. Must be from 10 to 60. If no rate is specified, the video's rate will be taken." << std::endl;
    std::cout << "-a or --accelerate: Use hardware acceleration." << std::endl;
    std::cout << "-s or --switch: Switch red and blue values.\n" << std::endl;
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

    Utils::UI::BagInputParameters inputParameters;

    // Video directory
    inputParameters.sourceDirectory = arguments.at(1);
    auto dirPath = inputParameters.sourceDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "The entered directory for the video file does not exist. Please specify a correct directory!" << std::endl;
        return 0;
    }
    const auto fileEnding = inputParameters.sourceDirectory.right(3);
    if (fileEnding != "mp4" && fileEnding != "mkv") {
        std::cerr << "The entered video name is not in correct format. Please make sure that the video file ends in mp4 or mkv!" << std::endl;
        return 0;
    }

    // Handle bag directory
    inputParameters.targetDirectory = arguments.at(2);
    dirPath = inputParameters.targetDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "Bag file not found. Make sure that the bag file exists!" << std::endl;
        return 0;
    }

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Topic name
        if (Utils::CLI::containsArguments(arguments, "-t", "--topic_name")) {
            const auto topicNameIndex = Utils::CLI::getArgumentsIndex(arguments, "-t", "--topic_name");
            if (arguments.at(topicNameIndex) == arguments.last()) {
                std::cerr << "Please enter a valid topic name!" << std::endl;
                return 0;
            }

            const auto& topicName = arguments.at(topicNameIndex + 1);
            if (!Utils::ROS::isNameROS2Conform(topicName)) {
                const auto errorString = "The topic name does not follow the ROS2 naming convention! More information on ROS2 naming convention is found here:\n"
                                         "https://design.ros2.org/articles/topic_and_service_names.html\n"
                                         "Do you want to continue anyways? [y/n]";
                if (!Utils::CLI::shouldContinue(errorString)) {
                    return 0;
                }
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
    }

    // Apply default topic name if not assigned
    if (inputParameters.topicName.isEmpty()) {
        inputParameters.topicName = "/topic_video";
    }

    if (std::filesystem::exists(inputParameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The bag file already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const writeToBagThread = new WriteToBagThread(inputParameters);

    QObject::connect(writeToBagThread, &WriteToBagThread::openingCVInstanceFailed, [] {
        std::cerr << "The video writing failed. Please make sure that all inputParameters are set correctly "
            "and disable the hardware acceleration, if necessary." << std::endl;
        return 0;
    });
    QObject::connect(writeToBagThread, &WriteToBagThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(writeToBagThread, &WriteToBagThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Writing finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(writeToBagThread, &WriteToBagThread::finished, writeToBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Writing video to bag. Please wait..." << std::endl;
    Utils::CLI::runThread(writeToBagThread, signalStatus);

    return EXIT_SUCCESS;
}
