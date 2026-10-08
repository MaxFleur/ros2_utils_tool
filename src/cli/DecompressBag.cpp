#include "ChangeCompressionBagThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>

#include <filesystem>
#include <iostream>

void
helpFunction()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_decompress_bag [-h] [bag_path] [output_decompressed_bag_path]\n";
    std::cout << "                                                    [--thread-count THREAD_COUNT] [-d] [-s]\n\n";
    std::cout << "Decompress a bag file.\n\n";
    std::cout << "positional arguments:\n";
    std::cout << "  bag_path              Source bag file, must be compressed.\n";
    std::cout << "  output_decompressed_bag_path\n";
    std::cout << "                        Decompressed bag file directory.\n\n";
    std::cout << "options:\n";
    std::cout << "  -h, --help            Show this help message and exit.\n";
    std::cout << "  --thread-count THREAD_COUNT\n";
    std::cout << "                        Number of threads used for decompression. Minimum is 1, maximum is " << std::thread::hardware_concurrency() << ", defaults to 1.\n";
    std::cout << "  -d, --delete          Delete the source file after completion.\n";
    std::cout << "  -s, --suppress        Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_decompress_bag /home/usr/compressed /home/usr/decompressed --thread-count 4" << std::endl;
}


int
main(int argc, char* argv[])
{
    // Don't want ROS logging to break our progress info
    Utils::ROS::disableROSLogging();
    // Create application
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (Utils::CLI::showHelpAndExitEarly(arguments, helpFunction, 3)) {
        return 0;
    }
    const QVector<QString> checkList { "-d", "-s", "--delete", "--suppress", "--thread-count" };
    Utils::CLI::checkForInvalidParameters(arguments, checkList, helpFunction);

    Parameters::DeleteSourceParameters parameters;

    // Compressed source bag directory
    parameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(parameters.sourceDirectory.toStdString())) {
        throw std::runtime_error("Source bag file not found. Make sure that the bag file exists!");
    }
    if (const auto alreadyCompressed = Utils::ROS::doesDirectoryContainCompressedBagFile(parameters.sourceDirectory); !alreadyCompressed) {
        throw std::runtime_error("The bag file is invalid or not in compressed format!");
    }

    // Compressed target bag directory
    parameters.targetDirectory = arguments.at(2);
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Delete source
        parameters.deleteSource = Utils::CLI::containsArguments(arguments, "-d", "--delete");
    }

    // Thread count
    auto numberOfThreads = 1;
    if (!Utils::CLI::checkArgumentValidity(arguments, "", "--thread-count", numberOfThreads, 1, std::thread::hardware_concurrency())) {
        throw std::runtime_error("Please enter a thread count value in the range of 1 to " + std::to_string(std::thread::hardware_concurrency()) + "!");
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and run the operation
    auto* const decompressBagThread = new ChangeCompressionBagThread(parameters, numberOfThreads, false);
    std::cout << "Source compressed bag file: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target decompressed bag file: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Number of used threads: " << numberOfThreads << "\n\n";
    Utils::CLI::runThread(decompressBagThread, "Decompressing finished!", Utils::CLI::ProgressMode::ProcessingSpinner);

    return EXIT_SUCCESS;
}
