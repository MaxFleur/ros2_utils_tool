#include "BagToPCDsThread.hpp"

#include "Parameters.hpp"
#include "UtilsCLI.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_pcds path/to/bag path/to/pcds\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-t or --topic_name: Point cloud topic inside the bag. If no topic name is specified, the first found point cloud topic in the bag is taken.\n\n";
    std::cout << "-th or --threads: Number of threads, must be at least 1 (maximum is " << std::thread::hardware_concurrency() << ").\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run mediassist4_ros_tools tool_bag_to_pcds /home/usr/input_bag /home/usr/pcd_dir -th 4\n\n";
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

    const QVector<QString> checkList{ "-t", "-th", "-s", "--topic_name", "--threads", "--suppress" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList);
        argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::AdvancedParameters parameters;

    // Handle bag directory
    parameters.sourceDirectory = arguments.at(1);
    Utils::CLI::checkBagSourceDirectory(parameters.sourceDirectory);

    // PCD files directory
    parameters.targetDirectory = arguments.at(2);
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Topic name
        Utils::CLI::checkTopicNameValidity(arguments, parameters.sourceDirectory, "sensor_msgs/msg/PointCloud2", parameters.topicName);
    }

    // Thread count
    auto numberOfThreads = 1;
    if (!Utils::CLI::checkArgumentValidity(arguments, "-th", "--threads", numberOfThreads, 1, std::thread::hardware_concurrency())) {
        throw std::runtime_error("Please enter a thread count value in the range of 1 to " + std::to_string(std::thread::hardware_concurrency()) + "!");
    }

    // Search for topic name in bag file if not specified
    if (parameters.topicName.isEmpty()) {
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, "sensor_msgs/msg/PointCloud2");
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and connect to its informations
    auto* const bagToPCDsThread = new BagToPCDsThread(parameters, numberOfThreads);
    QObject::connect(bagToPCDsThread, &BagToPCDsThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(bagToPCDsThread, &BagToPCDsThread::finished, [] {
        std::cout << "\n"; // Extra line to stop flushing
        std::cout << "Writing pcds finished!\n";
        return EXIT_SUCCESS;
    });
    QObject::connect(bagToPCDsThread, &BagToPCDsThread::finished, bagToPCDsThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Source bag file: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target pcd dir: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n";
    std::cout << "Number of used threads: " << numberOfThreads << "\n\n";
    std::cout << "Writing pcds. Please wait...\n";
    Utils::CLI::runThread(bagToPCDsThread, signalStatus);

    return EXIT_SUCCESS;
}
