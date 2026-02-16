#include "VideoToBagThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_video_to_bag path/to/video path/to/bag\n\n";
    std::cout << "The video must have an ending of .mp4 or .mkv.\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-r or --rate: Framerate for the image stream. Minimum is 10, maximum is 60, default is original video's rate.\n";
    std::cout << "-t or --topic_name: Topic name. If this is empty, the name '/topic_video' will be taken.\n\n";
    std::cout << "-a or --accelerate: Use hardware acceleration.\n";
    std::cout << "-e or --exchange: Exchange red and blue values.\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_video_to_bag /home/usr/video.mkv /home/usr/output_bag -t /example_topic -r 20 -a -s\n\n";
    std::cout << "-h or --help: Show this help.\n";
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    const QVector<QString> checkList{ "-r", "-t", "-a", "-e", "-s", "--rate", "--topic_name", "--accelerate", "--exchange", "--suppress" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::VideoToBagParameters parameters;

    // Video directory
    parameters.sourceDirectory = arguments.at(1);
    Utils::CLI::checkParentDirectory(parameters.sourceDirectory, false);

    if (const auto fileEnding = parameters.sourceDirectory.right(3); fileEnding != "mp4" && fileEnding != "mkv") {
        throw std::runtime_error("The entered video name is in invalid format. Please make sure that the video has the ending 'mp4' or 'mkv'!");
    }

    // Bag directory
    parameters.targetDirectory = arguments.at(2);
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);

    // Check for optional arguments
    auto useHardwareAcceleration = false;

    if (arguments.size() > 3) {
        // Topic name
        if (!Utils::CLI::continueWithInvalidROS2Name(arguments, parameters.topicName)) {
            return 0;
        }
        // Framerate
        parameters.useCustomFPS = Utils::CLI::containsArguments(arguments, "-r", "--rate");
        if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", parameters.fps, 10, 60)) {
            throw std::runtime_error("Please enter a framerate in the range of 10 to 60!");
        }
        // Hardware acceleration
        useHardwareAcceleration = Utils::CLI::containsArguments(arguments, "-a", "--accelerate");
        // Exchange red and blue values
        parameters.exchangeRedBlueValues = Utils::CLI::containsArguments(arguments, "-e", "--exchange");
    }

    // Apply default topic name if not assigned
    if (parameters.topicName.isEmpty()) {
        parameters.topicName = "/topic_video";
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and connect to its informations
    auto* const videoToBagThread = new VideoToBagThread(parameters, useHardwareAcceleration);

    QObject::connect(videoToBagThread, &VideoToBagThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(videoToBagThread, &VideoToBagThread::finished, [] {
        std::cout << "\n";// Extra line to stop flushing
        std::cout << "Writing finished!\n";
        return EXIT_SUCCESS;
    });
    QObject::connect(videoToBagThread, &VideoToBagThread::finished, videoToBagThread, &QObject::deleteLater);
    QObject::connect(videoToBagThread, &VideoToBagThread::failed, [] {
        throw std::runtime_error("Bag creation failed. Please make sure that all parameters are set correctly and disable the hardware acceleration, if necessary.");
    });

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Source video file: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target bag file: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n";
    std::cout << "Rate: " << parameters.fps << " fps\n\n";
    std::cout << "Please wait...\n";
    Utils::CLI::runThread(videoToBagThread, signalStatus);

    return EXIT_SUCCESS;
}
