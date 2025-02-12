#include "WriteToImageThread.hpp"

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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_images path/to/ROSBag path/to/target/image/dir\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Video topic inside the bag. If no topic name is specified, the first found video topic in the bag is taken.\n" << std::endl;
    std::cout << "-f or --format: Must be jpg, png or bmp (jpg is default)." << std::endl;
    std::cout << "-e or --exchange: Exchange red and blue values." << std::endl;
    std::cout << "-c or --colorless: Encode images without color.\n" << std::endl;
    std::cout << "-q 0-9 or --quality 0-9 (jpg and png only): Image quality, must be between 0 and 9 (9 is highest, 8 is default)." << std::endl;
    std::cout << "-o or --optimize (jpg only): Optimize jpg file size." << std::endl;
    std::cout << "-b or --binary (png only): Write images with only black and white pixels.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

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

    Utils::UI::ImageInputParameters inputParameters;

    // Handle bag directory
    inputParameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(inputParameters.sourceDirectory.toStdString())) {
        std::cerr << "Bag file not found. Make sure that the bag file exists!" << std::endl;
        return 0;
    }
    if (const auto doesDirContainBag = Utils::ROS::doesDirectoryContainBagFile(inputParameters.sourceDirectory); !doesDirContainBag) {
        std::cerr << "The directory does not contain a bag file!" << std::endl;
        return 0;
    }

    // Images directory
    inputParameters.targetDirectory = arguments.at(2);

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

        // Quality
        if (!Utils::CLI::checkArgumentValidity(arguments, "-q", "--quality", inputParameters.quality, 0, 9)) {
            std::cerr << "Please enter a quality value in the range of 0 to 9!" << std::endl;
            return 0;
        }
        // Format
        if (Utils::CLI::containsArguments(arguments, "-f", "--format")) {
            const auto qualityFormatIndex = Utils::CLI::getArgumentsIndex(arguments, "-f", "--format");
            if (arguments.at(qualityFormatIndex) == arguments.last() ||
                (arguments.at(qualityFormatIndex + 1) != "jpg" && arguments.at(qualityFormatIndex + 1) != "png" && arguments.at(qualityFormatIndex + 1) != "bmp")) {
                std::cerr << "Please enter either 'jpg', 'png' or 'bmp' for the format!" << std::endl;
                return 0;
            }
            inputParameters.format = arguments.at(qualityFormatIndex + 1);
        }

        // Exchange red and blue values
        inputParameters.exchangeRedBlueValues = Utils::CLI::containsArguments(arguments, "-e", "--exchange");
        // Black white images
        inputParameters.useBWImages = Utils::CLI::containsArguments(arguments, "-c", "--colorless");
        // Optimize for jpeg
        inputParameters.jpgOptimize = inputParameters.format == "jpg" && Utils::CLI::containsArguments(arguments, "-o", "--optimize");
        // Bilevel for png images
        inputParameters.pngBilevel = inputParameters.format == "png" && Utils::CLI::containsArguments(arguments, "-b", "--binary");
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
        if (!Utils::CLI::shouldContinue("The image directory already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const writeToImageThread = new WriteToImageThread(inputParameters);
    QObject::connect(writeToImageThread, &WriteToImageThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(writeToImageThread, &WriteToImageThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Writing images finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(writeToImageThread, &WriteToImageThread::finished, writeToImageThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Writing images. Please wait..." << std::endl;
    Utils::CLI::runThread(writeToImageThread, signalStatus);

    return EXIT_SUCCESS;
}
