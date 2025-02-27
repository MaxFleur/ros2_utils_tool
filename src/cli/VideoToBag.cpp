#include "VideoToBagThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"
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
    std::cout << "-e or --exchange: Exchange red and blue values.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto arguments = app.arguments();
    const QStringList checkList{ "-t", "-r", "-a", "-e", "-h", "--topic_name", "--rate", "--accelerate", "--exchange", "--help" };
    if (Utils::CLI::containsInvalidParameters(arguments, checkList) ||
        arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Parameters::VideoToBagParameters parameters;

    // Video directory
    parameters.sourceDirectory = arguments.at(1);
    auto dirPath = parameters.sourceDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "The entered directory for the video file does not exist. Please specify a correct directory!" << std::endl;
        return 0;
    }
    const auto fileEnding = parameters.sourceDirectory.right(3);
    if (fileEnding != "mp4" && fileEnding != "mkv") {
        std::cerr << "The entered video name is not in correct format. Please make sure that the video file ends in mp4 or mkv!" << std::endl;
        return 0;
    }

    // Handle bag directory
    parameters.targetDirectory = arguments.at(2);
    dirPath = parameters.targetDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "Invalid target directory. Please enter a valid one!" << std::endl;
        return 0;
    }

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Topic name
        if (!Utils::CLI::continueWithInvalidROS2Name(arguments, parameters.topicName)) {
            return 0;
        }
        // Framerate
        parameters.useCustomFPS = Utils::CLI::containsArguments(arguments, "-r", "--rate");
        if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", parameters.fps, 10, 60)) {
            std::cerr << "Please enter a framerate in the range of 10 to 60!" << std::endl;
            return 0;
        }
        // Hardware acceleration
        parameters.useHardwareAcceleration = Utils::CLI::containsArguments(arguments, "-a", "--accelerate");
        // Exchange red and blue values
        parameters.exchangeRedBlueValues = Utils::CLI::containsArguments(arguments, "-e", "--exchange");
    }

    // Apply default topic name if not assigned
    if (parameters.topicName.isEmpty()) {
        parameters.topicName = "/topic_video";
    }

    if (std::filesystem::exists(parameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The bag file already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const videoToBagThread = new VideoToBagThread(parameters);

    QObject::connect(videoToBagThread, &VideoToBagThread::openingCVInstanceFailed, [] {
        std::cerr << "The bag creation failed. Please make sure that all parameters are set correctly "
            "and disable the hardware acceleration, if necessary." << std::endl;
        return 0;
    });
    QObject::connect(videoToBagThread, &VideoToBagThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(videoToBagThread, &VideoToBagThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Writing finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(videoToBagThread, &VideoToBagThread::finished, videoToBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Writing video to bag. Please wait..." << std::endl;
    Utils::CLI::runThread(videoToBagThread, signalStatus);

    return EXIT_SUCCESS;
}
