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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_compress_bag path/to/uncompressed/source/bag /path/to/compressed/target/bag \n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-m or --mode (file/message): Compress per file (file) or per message (message). File is default.\n\n";
    std::cout << "-d or --delete: Delete the source file after completion.\n\n";
    std::cout << "-th or --threads: Number of threads, must be at least 1 (maximum is " << std::thread::hardware_concurrency() << ").\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "-h or --help: Show this help.\n";
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

    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, { "-m", "-d", "-th", "-s", "--mode", "--delete", "--threads", "--suppress" });
        argument != std::nullopt) {
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
    auto* const compressBagThread = new ChangeCompressionBagThread(parameters, numberOfThreads, true);
    auto isCompressing = false;
    std::thread processingThread;

    QObject::connect(compressBagThread, &ChangeCompressionBagThread::processing, [&processingThread, &isCompressing] {
        processingThread = std::thread(Utils::CLI::showProcessingString, std::ref(isCompressing), Utils::CLI::CLI_COMPRESS);

        return EXIT_SUCCESS;
    });
    QObject::connect(compressBagThread, &ChangeCompressionBagThread::finished, [&isCompressing, &processingThread] {
        isCompressing = false;
        processingThread.join();

        std::cout << "\n";// Extra line to stop flushing
        std::cout << "Compressing finished!\n";
        return EXIT_SUCCESS;
    });
    QObject::connect(compressBagThread, &ChangeCompressionBagThread::finished, compressBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });
    Utils::CLI::runThread(compressBagThread, signalStatus);

    return EXIT_SUCCESS;
}
