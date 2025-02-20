#include "PublishVideoThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_publish_video path/to/video\n" << std::endl;
    std::cout << "The video must have an ending of .mp4 or .mkv." << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Topic name. If this is empty, the name '/topic_video' will be taken.\n" << std::endl;
    std::cout << "-s width height or --scale width height: Scale. width must be between 1 and 3840, height between 1 and 2160.\n" << std::endl;
    std::cout << "-a or --accelerate: Use hardware acceleration." << std::endl;
    std::cout << "-e or --exchange: Exchange red and blue values." << std::endl;
    std::cout << "-l or --loop: Loop the video.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Initialize ROS and Qt
    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);

    const auto arguments = app.arguments();
    if (arguments.size() < 2 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Utils::UI::PublishParameters publishParameters;

    // Video directory
    publishParameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(publishParameters.sourceDirectory.toStdString())) {
        std::cerr << "The video file does not exist. Please enter a valid video path!" << std::endl;
        return 0;
    }
    const auto fileEnding = publishParameters.sourceDirectory.right(3);
    if (fileEnding != "mp4" && fileEnding != "mkv") {
        std::cerr << "The entered video name is not in correct format. Please make sure that the video file ends in mp4 or mkv!" << std::endl;
        return 0;
    }

    // Check for optional arguments
    if (arguments.size() > 2) {
        // Topic name
        if (!Utils::CLI::continueWithInvalidROS2Name(arguments, publishParameters.topicName)) {
            return 0;
        }
        // Scale
        publishParameters.scale = Utils::CLI::containsArguments(arguments, "-s", "--scale");
        if (!Utils::CLI::checkArgumentValidity(arguments, "-s", "--scale", publishParameters.width, 1, 3840)) {
            std::cerr << "Please enter a width value between 1 and 3840!" << std::endl;
            return 0;
        }
        if (!Utils::CLI::checkArgumentValidity(arguments, "-s", "--scale", publishParameters.height, 1, 2160, 2)) {
            std::cerr << "Please enter a height value between 1 and 2160!" << std::endl;
            return 0;
        }
        publishParameters.scale = true;
        // Hardware acceleration
        publishParameters.useHardwareAcceleration = Utils::CLI::containsArguments(arguments, "-a", "--accelerate");
        // Exchange red and blue values
        publishParameters.exchangeRedBlueValues = Utils::CLI::containsArguments(arguments, "-e", "--exchange");
        // Loop
        publishParameters.loop = Utils::CLI::containsArguments(arguments, "-l", "--loop");
    }

    // Apply default topic name if not assigned
    if (publishParameters.topicName.isEmpty()) {
        publishParameters.topicName = "/topic_video";
    }

    // Create thread and connect to its informations
    auto* const publishVideoThread = new PublishVideoThread(publishParameters);
    QObject::connect(publishVideoThread, &PublishVideoThread::openingCVInstanceFailed, [] {
        std::cerr << "Video publishing failed. Please make sure that the video file is valid and disable the hardware acceleration, if necessary." << std::endl;
        return 0;
    });
    QObject::connect(publishVideoThread, &PublishVideoThread::progressChanged, [] (const QString& progressString, int /* progress */) {
        std::cout << progressString.toStdString() << "\r" << std::flush;
    });
    QObject::connect(publishVideoThread, &PublishVideoThread::finished, publishVideoThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Publishing video..." << std::endl;
    Utils::CLI::runThread(publishVideoThread, signalStatus);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
