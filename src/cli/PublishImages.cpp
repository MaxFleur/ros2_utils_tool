#include "PublishImagesThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"

#include <QCoreApplication>

#include "rclcpp/rclcpp.hpp"

#include <opencv2/imgcodecs.hpp>

#include <filesystem>
#include <iostream>

void
helpFunction()
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
    if (Utils::CLI::showHelpAndExitEarly(arguments, helpFunction, 2)) {
        return 0;
    }

    const QVector<QString> checkList{ "-r", "-t", "-e", "-l", "-s", "--rate", "--topic", "--exchange", "--loop", "--suppress", "--scale" };
    Utils::CLI::checkForInvalidParameters(arguments, checkList, helpFunction);

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
        // Exchange red and blue values
        parameters.exchangeRedBlueValues = Utils::CLI::containsArguments(arguments, "-e", "--exchange");
        // Loop
        parameters.loop = Utils::CLI::containsArguments(arguments, "-l", "--loop");
    }

    // Apply default topic name if not assigned
    if (parameters.topicName.isEmpty()) {
        parameters.topicName = "/topic_video";
    }
    // Assign correct width and height values
    if (!parameters.scale) {
        for (auto const& entry : std::filesystem::directory_iterator(parameters.sourceDirectory.toStdString())) {
            if (entry.path().extension() != ".jpg" && entry.path().extension() != ".png" && entry.path().extension() != ".bmp") {
                continue;
            }
            const auto image = cv::imread(entry.path().string(), cv::IMREAD_COLOR);
            if (!image.empty()) {
                parameters.width = image.cols;
                parameters.height = image.rows;
            }
            break;
        }
    }

    // Create thread and run the operation
    auto* const publishImagesThread = new PublishImagesThread(parameters);
    std::cout << "Source images directory " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n";
    std::cout << "Images resolution: " << parameters.width << " x " << parameters.height << "\n";
    std::cout << "Rate: " << parameters.fps << " fps\n";
    if (parameters.loop) {
        std::cout << "Looping enabled.\n";
    }
    std::cout << "\n";
    Utils::CLI::runThread(publishImagesThread, "", Utils::CLI::ProgressMode::ProgressStringOnly, "Images publishing failed. Please make sure that the image files are valid!");

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
