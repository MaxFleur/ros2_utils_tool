#include "PublishVideoThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_publish_video path/to/video\n\n";
    std::cout << "The video must have an ending of .mp4 or .mkv.\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-t or --topic_name: Topic name. If this is empty, the name '/topic_video' will be taken.\n";
    std::cout << "-sc width height or --scale width height: Scale. width must be between 1 and 3840, height between 1 and 2160.\n\n";
    std::cout << "-a or --accelerate: Use hardware acceleration.\n";
    std::cout << "-e or --exchange: Exchange red and blue values.\n";
    std::cout << "-l or --loop: Loop the video.\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run mediassist4_ros_tools tool_publish_video /home/usr/video.mkv -sc 1280 720 -t /video_scaled -l\n\n";
    std::cout << "-h or --help: Show this help.\n";
}


int
main(int argc, char* argv[])
{
    // Initialize ROS and Qt
    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (arguments.size() < 2 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    const QVector<QString> checkList{ "-t", "-sc", "-a", "-e", "-l", "-s", "--topic_name", "--scale", "--accelerate", "--exchange", "--loop", "--suppress" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::PublishParameters parameters;

    // Video directory
    parameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(parameters.sourceDirectory.toStdString())) {
        throw std::runtime_error("The video file does not exist. Please enter a valid video path!");
    }
    const auto fileEnding = parameters.sourceDirectory.right(3);
    if (fileEnding != "mp4" && fileEnding != "mkv") {
        throw std::runtime_error("The entered video name is not in correct format. Please make sure that the video file ends in mp4 or mkv!");
    }

    // Check for optional arguments
    auto useHardwareAcceleration = false;

    if (arguments.size() > 2) {
        // Topic name
        if (!Utils::CLI::continueWithInvalidROS2Name(arguments, parameters.topicName)) {
            return 0;
        }
        // Scale
        parameters.scale = Utils::CLI::containsArguments(arguments, "-sc", "--scale");
        if (!Utils::CLI::checkArgumentValidity(arguments, "-sc", "--scale", parameters.width, 1, 3840)) {
            throw std::runtime_error("Please enter a width value between 1 and 3840!");
        }
        if (!Utils::CLI::checkArgumentValidity(arguments, "-sc", "--scale", parameters.height, 1, 2160, 2)) {
            throw std::runtime_error("Please enter a height value between 1 and 2160!");
        }
        parameters.scale = true;
        // Hardware acceleration
        useHardwareAcceleration = Utils::CLI::containsArguments(arguments, "-a", "--accelerate");
        // Exchange red and blue values
        parameters.exchangeRedBlueValues = Utils::CLI::containsArguments(arguments, "-e", "--exchange");
        // Loop
        parameters.loop = Utils::CLI::containsArguments(arguments, "-l", "--loop");
    }

    // Apply default topic name if not assigned
    if (parameters.topicName.isEmpty()) {
        parameters.topicName = "/topic_video";
    }

    // Create thread and connect to its informations
    auto* const publishVideoThread = new PublishVideoThread(parameters, useHardwareAcceleration);
    QObject::connect(publishVideoThread, &PublishVideoThread::progressChanged, [] (const QString& progressString, int /* progress */) {
        std::cout << progressString.toStdString() << "\r" << std::flush;
    });
    QObject::connect(publishVideoThread, &PublishVideoThread::finished, publishVideoThread, &QObject::deleteLater);
    QObject::connect(publishVideoThread, &PublishVideoThread::failed, [] {
        throw std::runtime_error("Video publishing failed. Please make sure that the video file is valid and disable the hardware acceleration, if necessary.");
    });

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Video file " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n";
    std::cout << "Resolution: " << parameters.width << " x " << parameters.height << "\n";
    if (parameters.loop) {
        std::cout << "Looping enabled.\n";
    }
    std::cout << "\n";
    Utils::CLI::runThread(publishVideoThread, signalStatus);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
