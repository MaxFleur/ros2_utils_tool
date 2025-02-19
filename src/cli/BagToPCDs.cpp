#include "WriteToPCDsThread.hpp"

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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_pcds path/to/ROSBag path/to/pcd/dir\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Point cloud topic inside the bag. If no topic name is specified, the first found point cloud topic in the bag is taken.\n" << std::endl;
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

    Utils::UI::AdvancedInputParameters inputParameters;

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
        if (!Utils::CLI::isTopicNameValid(arguments, inputParameters.sourceDirectory, "sensor_msgs/msg/PointCloud2", inputParameters.topicName)) {
            return 0;
        }
    }

    // Search for topic name in bag file if not specified
    if (inputParameters.topicName.isEmpty()) {
        const auto& firstTopicWithImageType = Utils::ROS::getFirstTopicWithCertainType(inputParameters.sourceDirectory, "sensor_msgs/msg/PointCloud2");
        if (firstTopicWithImageType == std::nullopt) {
            std::cerr << "The bag file does not contain any point cloud topics!" << std::endl;
            return 0;
        }

        inputParameters.topicName = *firstTopicWithImageType;
    }

    if (std::filesystem::exists(inputParameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The target directory already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const writeToPCDsThread = new WriteToPCDsThread(inputParameters);
    QObject::connect(writeToPCDsThread, &WriteToPCDsThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(writeToPCDsThread, &WriteToPCDsThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Writing images finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(writeToPCDsThread, &WriteToPCDsThread::finished, writeToPCDsThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Writing images. Please wait..." << std::endl;
    Utils::CLI::runThread(writeToPCDsThread, signalStatus);

    return EXIT_SUCCESS;
}
