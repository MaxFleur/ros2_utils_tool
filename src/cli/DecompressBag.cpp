#include "ChangeCompressionBagThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_decompress_bag path/to/compressed/source/bag /path/to/uncompressed/target/bag \n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-d or --delete: Delete the source file after completion.\n\n";
    std::cout << "-th or --threads: Number of threads, must be at least 1 (maximum is " << std::thread::hardware_concurrency() << ").\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run mediassist4_ros_tools tool_decompress_bag /home/usr/compressed /home/usr/decompressed -th 4\n\n";
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
    const QVector<QString> checkList { "-d", "-th", "-s", "--delete", "--threads", "--suppress" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList);
        argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::CompressBagParameters parameters;

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
    if (!Utils::CLI::checkArgumentValidity(arguments, "-th", "--threads", numberOfThreads, 1, std::thread::hardware_concurrency())) {
        throw std::runtime_error("Please enter a thread count value in the range of 1 to " + std::to_string(std::thread::hardware_concurrency()) + "!");
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and connect to its informations
    auto* const decompressBagThread = new ChangeCompressionBagThread(parameters, numberOfThreads, false);
    auto isCompressing = false;
    std::thread processingThread;

    QObject::connect(decompressBagThread, &ChangeCompressionBagThread::processing, [&processingThread, &isCompressing] {
        processingThread = std::thread(Utils::CLI::showProcessingString, std::ref(isCompressing));

        return EXIT_SUCCESS;
    });
    QObject::connect(decompressBagThread, &ChangeCompressionBagThread::finished, [&isCompressing, &processingThread] {
        isCompressing = false;
        processingThread.join();

        std::cout << "\n"; // Extra line to stop flushing
        std::cout << "Decompressing finished!\n";
        return EXIT_SUCCESS;
    });
    QObject::connect(decompressBagThread, &ChangeCompressionBagThread::finished, decompressBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Source compressed bag file: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target decompressed bag file: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Number of used threads: " << numberOfThreads << "\n\n";
    Utils::CLI::runThread(decompressBagThread, signalStatus);

    return EXIT_SUCCESS;
}
