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
    std::cout << "-d or --delete: Delete the source file after completion.\n" << std::endl;
    std::cout << "-s or --suppress: Suppress any warnings.\n" << std::endl;
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
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, { "-d", "-s", "--delete", "--suppress" }); argument != std::nullopt) {
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

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
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
