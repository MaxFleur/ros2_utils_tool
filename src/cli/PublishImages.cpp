#include "PublishImagesThread.hpp"

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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_publish_images path/to/images\n\n";
    std::cout << "The images must have format jpg, png or bmp.\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-t or --topic_name: Topic name. If this is empty, the name '/topic_video' will be taken.\n";
    std::cout << "-r or --rate: Framerate for the published video. Must be from 1 to 60, default is 30.\n";
    std::cout << "-sc width height or --scale width height: Scale. width must be between 1 and 3840, height between 1 and 2160.\n\n";
    std::cout << "-e or --exchange: Exchange red and blue values.\n";
    std::cout << "-l or --loop: Loop the video.\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run mediassist4_ros_tools tool_publish_images /home/usr/images_dir -sc 1280 720 -t /images_scaled -r 25 -l\n\n";
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

    const QVector<QString> checkList{ "-t", "-sc", "-r", "-e", "-l", "-s", "--topic_name", "--scale", "--rate", "--exchange", "--loop", "--suppress" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::PublishParameters parameters;

    // Images directory
    parameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(parameters.sourceDirectory.toStdString())) {
        throw std::runtime_error("The images directory does not exist. Please enter a valid images path!");
    }
    auto containsImageFiles = false;
    for (auto const& entry : std::filesystem::directory_iterator(parameters.sourceDirectory.toStdString())) {
        if (entry.path().extension() == ".jpg" || entry.path().extension() == ".png" || entry.path().extension() == ".bmp") {
            containsImageFiles = true;
            break;
        }
    }
    if (!containsImageFiles) {
        throw std::runtime_error("The specified directory does not contain any images!");
    }

    // Check for optional arguments
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
        parameters.scale = Utils::CLI::containsArguments(arguments, "-sc", "--scale");
        if (!Utils::CLI::checkArgumentValidity(arguments, "-sc", "--scale", parameters.width, 1, 3840)) {
            throw std::runtime_error("Please enter a width value between 1 and 3840!");
        }
        if (!Utils::CLI::checkArgumentValidity(arguments, "-sc", "--scale", parameters.height, 1, 2160, 2)) {
            throw std::runtime_error("Please enter a height value between 1 and 2160!");
        }
        parameters.scale = true;
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
    auto* const publishImagesThread = new PublishImagesThread(parameters);
    QObject::connect(publishImagesThread, &PublishImagesThread::progressChanged, [] (const QString& progressString, int /* progress */) {
        std::cout << progressString.toStdString() << "\r" << std::flush;
    });
    QObject::connect(publishImagesThread, &PublishImagesThread::finished, publishImagesThread, &QObject::deleteLater);
    QObject::connect(publishImagesThread, &PublishImagesThread::failed, [] {
        throw std::runtime_error("Images publishing failed. Please make sure that the image files are valid!");
    });

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Source images directory " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n";
    std::cout << "Images resolution: " << parameters.width << " x " << parameters.height << "\n";
    std::cout << "Rate: " << parameters.fps << " fps\n";
    if (parameters.loop) {
        std::cout << "Looping enabled.\n";
    }
    std::cout << "\n";
    Utils::CLI::runThread(publishImagesThread, signalStatus);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
