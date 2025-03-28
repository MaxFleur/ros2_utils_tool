#include "BagToPCDsThread.hpp"

#include "Parameters.hpp"
#include "UtilsCLI.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_pcds path/to/bag path/to/pcds\n" << std::endl;
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
    const QStringList checkList{ "-t", "-h", "--topic_name", "--help" };
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::AdvancedParameters parameters;

    // Handle bag directory
    parameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(parameters.sourceDirectory.toStdString())) {
        throw std::runtime_error("Bag file not found. Make sure that the bag file exists!");
    }
    if (const auto doesDirContainBag = Utils::ROS::doesDirectoryContainBagFile(parameters.sourceDirectory); !doesDirContainBag) {
        throw std::runtime_error("The directory does not contain a bag file!");
    }

    // PCD files directory
    parameters.targetDirectory = arguments.at(2);
    auto dirPath = parameters.targetDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        throw std::runtime_error("Invalid target directory. Please enter a valid one!");
    }

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Topic name
        Utils::CLI::checkTopicNameValidity(arguments, parameters.sourceDirectory, "sensor_msgs/msg/PointCloud2", parameters.topicName);
    }

    // Search for topic name in bag file if not specified
    if (parameters.topicName.isEmpty()) {
        const auto& firstTopicWithImageType = Utils::ROS::getFirstTopicWithCertainType(parameters.sourceDirectory, "sensor_msgs/msg/PointCloud2");
        if (firstTopicWithImageType == std::nullopt) {
            throw std::runtime_error("The bag file does not contain any point cloud topics!");
        }

        parameters.topicName = *firstTopicWithImageType;
    }

    if (std::filesystem::exists(parameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The target directory already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const bagToPCDsThread = new BagToPCDsThread(parameters, std::thread::hardware_concurrency());
    QObject::connect(bagToPCDsThread, &BagToPCDsThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(bagToPCDsThread, &BagToPCDsThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Writing pcds finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(bagToPCDsThread, &BagToPCDsThread::finished, bagToPCDsThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Writing pcds. Please wait..." << std::endl;
    Utils::CLI::runThread(bagToPCDsThread, signalStatus);

    return EXIT_SUCCESS;
}
