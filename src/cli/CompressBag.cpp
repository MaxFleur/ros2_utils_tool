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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_compress_bag path/to/uncompressed/source/bag /path/to/compressed/target/bag \n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-m or --mode (file/message): Compress per file (file) or per message (message). File is default." << std::endl;
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
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, { "-m", "--mode", "-k", "--keep" }); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::CompressBagParameters parameters;

    // Uncompressed source bag directory
    parameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(parameters.sourceDirectory.toStdString())) {
        throw std::runtime_error("Source bag file not found. Make sure that the bag file exists!");
    }
    if (const auto alreadyCompressed = Utils::ROS::doesDirectoryContainCompressedBagFile(parameters.sourceDirectory); alreadyCompressed) {
        throw std::runtime_error("The source bag file already compressed!");
    }
    if (const auto doesDirContainBag = Utils::ROS::doesDirectoryContainBagFile(parameters.sourceDirectory); !doesDirContainBag) {
        throw std::runtime_error("The source bag file is invalid!");
    }

    // Target compressed bag directory
    parameters.targetDirectory = arguments.at(2);
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);

    parameters.deleteSource = true;
    // Check for optional arguments
    if (arguments.size() > 3) {
        // Mode
        if (Utils::CLI::containsArguments(arguments, "-m", "--mode")) {
            const auto modeIndex = Utils::CLI::getArgumentsIndex(arguments, "-m", "--mode");
            if (arguments.at(modeIndex) == arguments.last() || (arguments.at(modeIndex + 1) != "file" && arguments.at(modeIndex + 1) != "message")) {
                throw std::runtime_error("Please enter either 'file' or 'message' for the mode!");
            }
            parameters.compressPerMessage = arguments.at(modeIndex + 1) == "message";
        }

        // Delete source
        parameters.deleteSource = !Utils::CLI::containsArguments(arguments, "-k", "--keep");
    }

    if (std::filesystem::exists(parameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The target directory already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const compressBagThread = new ChangeCompressionBagThread(parameters, std::thread::hardware_concurrency(), true);
    auto isCompressing = false;
    std::thread processingThread;

    QObject::connect(compressBagThread, &ChangeCompressionBagThread::processing, [&processingThread, &isCompressing] {
        processingThread = std::thread(Utils::CLI::showProcessingString, std::ref(isCompressing), Utils::CLI::CLI_COMPRESS);

        return EXIT_SUCCESS;
    });
    QObject::connect(compressBagThread, &ChangeCompressionBagThread::finished, [&isCompressing, &processingThread] {
        isCompressing = false;
        processingThread.join();

        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Compressing finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(compressBagThread, &ChangeCompressionBagThread::finished, compressBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });
    Utils::CLI::runThread(compressBagThread, signalStatus);

    return EXIT_SUCCESS;
}
