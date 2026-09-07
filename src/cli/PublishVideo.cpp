#include "PublishVideoThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"

#include <QCoreApplication>
#include <QObject>

#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_publish_video [-h] [video_dir] [--scale WIDTH HEIGHT] [-r RATE] [-t TOPIC] [-a] [-e] [-l] [-s]\n\n";
    std::cout << "Publish a stored video as a ROS2 image messages stream. The video must have a format of mp4 or mkv.\n\n";
    std::cout << "positional arguments:\n";
    std::cout << "  video_dir             Source video directory.\n\n";
    std::cout << "options:\n";
    std::cout << "  -h, --help            Show this help message and exit.\n";
    std::cout << "  --scale WIDTH HEIGHT\n";
    std::cout << "                        Scale the video to a new resolution. WIDTH must be in the range of 1 to 3840, HEIGHT of 1 to 2160.\n";
    std::cout << "  -r RATE, --rate RATE  Number of messages per second. Minimum is 1, maximum is 60, defaults to 30.\n";
    std::cout << "  -t TOPIC, --topic TOPIC\n";
    std::cout << "                        Image messages topic name, defaults to '/topic_video'.\n";
    std::cout << "  -a, --accelerate      Use hardware acceleration.\n";
    std::cout << "  -e, --exchange        Exchange red and blue values.\n";
    std::cout << "  -l, --loop            Loop the video.\n";
    std::cout << "  -s, --suppress        Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_publish_video /home/usr/video.mkv --scale 1280 720 -t /video_scaled -l" << std::endl;
}


int
main(int argc, char* argv[])
{
    // Initialize ROS and Qt
    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (arguments.size() < 2 || Utils::CLI::containsArguments(arguments, "-h", "--help")) {
        showHelp();
        return 0;
    }

    const QVector<QString> checkList{ "-r", "-t", "-a", "-e", "-l", "-s",
                                      "--scale", "--rate", "--topic", "--accelerate", "--exchange", "--loop", "--suppress" };
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
        // Framerate
        if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", parameters.fps, 1, 60)) {
            throw std::runtime_error("Please enter a framerate in the range of 1 to 60!");
        }
        // Scale
        parameters.scale = arguments.contains("--scale");
        if (!Utils::CLI::checkArgumentValidity(arguments, "", "--scale", parameters.width, 1, 3840)) {
            throw std::runtime_error("Please enter a width value between 1 and 3840!");
        }
        if (!Utils::CLI::checkArgumentValidity(arguments, "", "--scale", parameters.height, 1, 2160, 2)) {
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
    // Start operation
    Utils::CLI::runThread(publishVideoThread, signalStatus);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
