#include "DummyBagThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>
#include <QSet>

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_dummy_bag path/to/bag topic_name_1 topic_type_1 (...)\n\n";
    std::cout << "Topic type is either 'String', 'Integer', 'Image', 'PointCloud' or 'TF2'.\n";
    std::cout << "You can write up to five topics.\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-m or --message-count: Number of messages in the bag file. Minimum is 1, maximum is 1000, default is 100.\n";
    std::cout << "-r or --rate: Number of messages per second. Minimum is 1, maximum is 100, default is 10.\n";
    std::cout << "-th or --threads: Number of threads. Minimum is 1, maximum is " << std::thread::hardware_concurrency() << ", default is 1.\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_dummy_bag /home/usr/target_bag /images Image /tf2 TF2 /int Integer\n\n";
    std::cout << "-h or --help: Show this help.\n";
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    // 20 means all five topics plus every possible flag
    if (arguments.size() < 4 || arguments.size() > 20 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    const QVector<QString> checkList{ "-m", "-r", "-th", "-s", "--message-count", "--rate", "--threads", "--suppress" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::DummyBagParameters parameters;

    // Bag directory (called as source dir, but is out target dir this time)
    parameters.sourceDirectory = arguments.at(1);
    Utils::CLI::checkParentDirectory(parameters.sourceDirectory);

    // Message count
    parameters.messageCount = 100;
    if (!Utils::CLI::checkArgumentValidity(arguments, "-m", "--message-count", parameters.messageCount, 1, 1000)) {
        throw std::runtime_error("Please enter a message count in the range of 1 to 1000!");
    }
    // Rate
    parameters.rate = 10;
    if (Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", parameters.rate, 1, 100)) {
        parameters.useCustomRate = true;
    } else {
        throw std::runtime_error("Please enter a rate in the range of 1 to 100!");
    }

    // Topics
    QVector<QString> topicTypes;
    QVector<QString> topicNames;
    QSet<QString> topicNameSet;
    auto areROS2NamesValid = true;

    auto isArgumentOptional = [checkList] (const QString& argument) {
        return checkList.contains(argument);
    };
    // Ensure correct topic type and name ordering
    for (auto i = 2; i < arguments.size() && !isArgumentOptional(arguments.at(i)); i++) {
        const auto argument = arguments.at(i);

        if (i % 2 == 0) {
            if (!Utils::ROS::isNameROS2Conform(argument) && areROS2NamesValid && !Utils::CLI::containsArguments(arguments, "-s", "--suppress")) {
                const auto errorString = "The topic name does not follow the ROS2 naming convention! More information on ROS2 naming convention is found here:\n"
                                         "https://design.ros2.org/articles/topic_and_service_names.html\n"
                                         "Do you want to continue anyways? [y]/n";
                if (!Utils::CLI::shouldContinue(errorString)) {
                    return 0;
                }
                areROS2NamesValid = false;
            }

            topicNames.push_back(argument);
            topicNameSet.insert(argument);
        } else {
            if (const QVector<QString> arguments { "String", "Integer", "Image", "PointCloud", "TF2" }; !arguments.contains(argument)) {
                throw std::runtime_error("The topic type must be either 'String', 'Integer', 'Image', 'PointCloud' or 'TF2'!");
            }
            topicTypes.push_back(argument);
        }
    }

    // Thread count
    auto numberOfThreads = 1;
    if (!Utils::CLI::checkArgumentValidity(arguments, "-th", "--threads", numberOfThreads, 1, std::thread::hardware_concurrency())) {
        throw std::runtime_error("Please enter a thread count value in the range of 1 to " + std::to_string(std::thread::hardware_concurrency()) + "!");
    }

    if (topicTypes.size() != topicNames.size()) {
        throw std::runtime_error("Topic type and topic name size do not match. Please make sure to enter a name for each topic!");
    }
    if (topicNameSet.size() != topicNames.size()) {
        throw std::runtime_error("Duplicate topic names detected. Please make sure that every topic name is unique!");
    }
    if (topicNames.size() > 5) {
        throw std::runtime_error("Please add a maximum of five topics!");
    }

    // Create thread parameters
    QVector<Parameters::DummyBagParameters::DummyBagTopic> topics;
    for (auto i = 0; i < topicTypes.size(); i++) {
        parameters.topics.push_back({ topicTypes.at(i), topicNames.at(i) });
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.sourceDirectory)) {
        return 0;
    }

    // Create thread and connect to its informations
    auto* const dummyBagThread = new DummyBagThread(parameters, numberOfThreads);
    std::mutex mutex;

    QObject::connect(dummyBagThread, &DummyBagThread::progressChanged, [&mutex] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        mutex.lock();
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
        mutex.unlock();
    });
    QObject::connect(dummyBagThread, &DummyBagThread::finished, [] {
        // This signal is thrown even if SIGINT is called, but we haven't finished, only interrupted
        if (signalStatus != SIGINT) {
            std::cout << "\n"; // Extra line to stop flushing
            std::cout << "Creating bag finished!\n";
        }
        return EXIT_SUCCESS;
    });
    QObject::connect(dummyBagThread, &DummyBagThread::finished, dummyBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Target bag file: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Topics to create:\n";
    for (const auto& topic : parameters.topics) {
        std::cout << "    " << topic.name.toStdString() << " (Type: " << topic.type.toStdString() << ")\n";
    }
    std::cout << "Message count: " << parameters.messageCount << "\n";
    std::cout << "Rate: " << parameters.rate << "\n";
    std::cout << "Number of used threads: " << numberOfThreads << "\n\n";
    std::cout << "Please wait...\n";
    Utils::CLI::runThread(dummyBagThread, signalStatus);

    return EXIT_SUCCESS;
}
