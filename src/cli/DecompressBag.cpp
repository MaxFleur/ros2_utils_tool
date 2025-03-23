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

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_decompress_bag path/to/compressed/source/bag /path/to/uncompressed/target/bag \n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-k or --keep: Keep the source file after completion." << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto arguments = app.arguments();
    if (Utils::CLI::containsInvalidParameters(arguments, { "-k", "--keep" }) ||
        arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Parameters::CompressBagParameters parameters;

    // Handle bag directory
    parameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(parameters.sourceDirectory.toStdString())) {
        std::cerr << "Source bag file not found. Make sure that the bag file exists!" << std::endl;
        return 0;
    }
    if (const auto alreadyCompressed = Utils::ROS::doesDirectoryContainCompressedBagFile(parameters.sourceDirectory); !alreadyCompressed) {
        std::cerr << "The bag file is invalid or not in compressed format!" << std::endl;
        return 0;
    }

    // Images directory
    parameters.targetDirectory = arguments.at(2);
    auto dirPath = parameters.targetDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "Invalid target directory. Please enter a valid one!" << std::endl;
        return 0;
    }

    parameters.deleteSource = true;
    // Check for optional arguments
    if (arguments.size() > 3) {
        // Delete source
        parameters.deleteSource = !Utils::CLI::containsArguments(arguments, "-k", "--keep");
    }

    if (std::filesystem::exists(parameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The target directory already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const decompressBagThread = new ChangeCompressionBagThread(parameters, std::thread::hardware_concurrency(), false);
    auto isCompressing = false;
    std::thread processingThread;

    QObject::connect(decompressBagThread, &ChangeCompressionBagThread::processing, [&processingThread, &isCompressing] {
        processingThread = std::thread(Utils::CLI::showProcessingString, std::ref(isCompressing), Utils::CLI::CLI_DECOMPRESS);

        return EXIT_SUCCESS;
    });
    QObject::connect(decompressBagThread, &ChangeCompressionBagThread::finished, [&isCompressing, &processingThread] {
        isCompressing = false;
        processingThread.join();

        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Decompressing finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(decompressBagThread, &ChangeCompressionBagThread::finished, decompressBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });
    Utils::CLI::runThread(decompressBagThread, signalStatus);

    return EXIT_SUCCESS;
}
