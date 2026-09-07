#include "PublishImagesThread.hpp"

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
    std::cout << "Usage: ros2 run ros2_utils_tool tool_publish_images [-h] [images_dir] [--scale WIDTH HEIGHT] [-r RATE] [-t TOPIC] [-e] [-l] [-s]\n\n";
    std::cout << "Publish a set of images as a ROS2 image messages stream. The images must have format jpg, png or bmp.\n\n";
    std::cout << "positional arguments:\n";
    std::cout << "  files_dir             Source files directory.\n";
    std::cout << "options:\n";
    std::cout << "  -h, --help            Show this help message and exit.\n";
    std::cout << "  --scale WIDTH HEIGHT\n";
    std::cout << "                        Scale the video to a new resolution. WIDTH must be in the range of 1 to 3840, HEIGHT of 1 to 2160.\n";
    std::cout << "  -r RATE, --rate RATE  Number of messages per second. Minimum is 1, maximum is 60, defaults to 30.\n";
    std::cout << "  -t TOPIC, --topic TOPIC\n";
    std::cout << "                        Image messages topic name, defaults to '/topic_video'.\n";
    std::cout << "  -e, --exchange        Exchange red and blue values.\n";
    std::cout << "  -l, --loop            Loop the video.\n";
    std::cout << "  -s, --suppress        Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_publish_images /home/usr/images_dir --scale 1280 720 -t /images_scaled -r 25 -l" << std::endl;
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

    const QVector<QString> checkList{ "-r", "-t", "-e", "-l", "-s", "--rate", "--topic", "--exchange", "--loop", "--suppress", "--scale" };
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
        parameters.scale = arguments.contains("--scale");
        if (!Utils::CLI::checkArgumentValidity(arguments, "", "--scale", parameters.width, 1, 3840)) {
            throw std::runtime_error("Please enter a width value between 1 and 3840!");
        }
        if (!Utils::CLI::checkArgumentValidity(arguments, "", "--scale", parameters.height, 1, 2160, 2)) {
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
    // Start operation
    Utils::CLI::runThread(publishImagesThread, signalStatus);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
