#include "CompressBagThread.hpp"

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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_compress_bag path/to/source/bag path/to/target/bag \n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-m or --mode (file/message): Compress per file (file) or per message (message). File is default." << std::endl;
    std::cout << "-d or --delete: Delete the source file after completion." << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto arguments = app.arguments();
    if (Utils::CLI::containsInvalidParameters(arguments, { "-m", "--mode", "-d", "--delete" }) ||
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
    if (const auto doesDirContainBag = Utils::ROS::doesDirectoryContainBagFile(parameters.sourceDirectory); !doesDirContainBag) {
        std::cerr << "The source directory does not contain a bag file!" << std::endl;
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

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Mode
        if (Utils::CLI::containsArguments(arguments, "-m", "--mode")) {
            const auto modeIndex = Utils::CLI::getArgumentsIndex(arguments, "-m", "--mode");
            if (arguments.at(modeIndex) == arguments.last() || (arguments.at(modeIndex + 1) != "file" && arguments.at(modeIndex + 1) != "message")) {
                std::cerr << "Please enter either 'file' or 'message' for the mode!" << std::endl;
                return 0;
            }
            parameters.compressPerMessage = arguments.at(modeIndex + 1) == "message";
        }

        // Delete source
        parameters.deleteSource = Utils::CLI::containsArguments(arguments, "-d", "--delete");
    }

    if (std::filesystem::exists(parameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The target directory already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const compressBagThread = new CompressBagThread(parameters, std::thread::hardware_concurrency());
    auto isCompressing = false;
    std::thread processingThread;

    QObject::connect(compressBagThread, &CompressBagThread::processing, [&processingThread, &isCompressing] {
        processingThread = std::thread(Utils::CLI::showIsProcessingString, std::ref(isCompressing), true);

        return EXIT_SUCCESS;
    });
    QObject::connect(compressBagThread, &CompressBagThread::finished, [&isCompressing, &processingThread] {
        isCompressing = false;
        processingThread.join();

        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Compressing finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(compressBagThread, &CompressBagThread::finished, compressBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });
    Utils::CLI::runThread(compressBagThread, signalStatus);

    return EXIT_SUCCESS;
}
