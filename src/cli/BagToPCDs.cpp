#include "BagToPCDsThread.hpp"

#include "Parameters.hpp"
#include "UtilsCLI.hpp"

#include <QCoreApplication>

#include <filesystem>
#include <iostream>

void
helpFunction()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_bag_to_pcds [-h] [bag_path] [output_files_path] [-t TOPIC]\n";
    std::cout << "                                                 [--thread-count THREAD_COUNT] [-s]\n\n";
    std::cout << "Convert bag point cloud messages to a list of files\n\n";
    std::cout << "positional arguments:\n";
    std::cout << "  bag_path              Source bag file.\n";
    std::cout << "  output_files_path     Directory containing the output image files.\n\n";
    std::cout << "options:\n";
    std::cout << "  -h, --help            Show this help message and exit.\n";
    std::cout << "  -t TOPIC, --topic TOPIC\n";
    std::cout << "                        Bag point cloud topic to convert.\n";
    std::cout << "                        If no topic name is specified, the first found topic with type point cloud is taken.\n";
    std::cout << "  --thread-count THREAD_COUNT\n";
    std::cout << "                        Number of threads used for writing. Minimum is 1, maximum is " << std::thread::hardware_concurrency() << ", defaults to 1.\n";
    std::cout << "  -s, --suppress        Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_bag_to_pcds /home/usr/input_bag /home/usr/pcd_dir --thread-count 4" << std::endl;
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (Utils::CLI::showHelpAndExitEarly(arguments, helpFunction, 3)) {
        return 0;
    }

    const QVector<QString> checkList{ "-t", "-s", "--topic", "--suppress", "--thread-count" };
    Utils::CLI::checkForInvalidParameters(arguments, checkList, helpFunction);

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
        Utils::CLI::checkTopicNameValidity(arguments, parameters.sourceDirectory, { "sensor_msgs/msg/PointCloud2" }, parameters.topicName);
    }

    // Thread count
    auto numberOfThreads = 1;
    if (!Utils::CLI::checkArgumentValidity(arguments, "", "--thread-count", numberOfThreads, 1, std::thread::hardware_concurrency())) {
        throw std::runtime_error("Please enter a thread count value in the range of 1 to " + std::to_string(std::thread::hardware_concurrency()) + "!");
    }

    // Search for topic name in bag file if not specified
    if (parameters.topicName.isEmpty()) {
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, { "sensor_msgs/msg/PointCloud2" });
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and run the operation
    auto* const bagToPCDsThread = new BagToPCDsThread(parameters, numberOfThreads);
    std::cout << "Source bag file: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target pcd dir: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n";
    std::cout << "Number of used threads: " << numberOfThreads << "\n\n";
    std::cout << "Writing pcds. Please wait...\n";
    Utils::CLI::runThread(bagToPCDsThread, "Writing pcds finished!");

    return EXIT_SUCCESS;
}
