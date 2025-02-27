#include "DummyBagThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>
#include <QSet>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_dummy_bag path/to/ROSBag topic_name_1 topic_type_1 "
        "(...) message_count\n" << std::endl;
    std::cout << "Topic type is either 'String', 'Integer', 'Image' or 'PointCloud'." << std::endl;
    std::cout << "You can write up to four topics." << std::endl;
    std::cout << "The message count must be between 1 and 1000.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto arguments = app.arguments();
    if (Utils::CLI::containsInvalidParameters(arguments, { "-h", "--help" }) ||
        arguments.size() < 4 || arguments.size() > 11 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Parameters::DummyBagParameters parameters;
    parameters.topicName = "";

    // Bag directory
    parameters.sourceDirectory = arguments.at(1);
    auto dirPath = parameters.sourceDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "Invalid target directory. Please enter a valid one!" << std::endl;
        return 0;
    }

    // Message count
    parameters.messageCount = arguments.at(arguments.size() - 1).toInt();
    if (parameters.messageCount < 1 || parameters.messageCount > 1000) {
        std::cerr << "Please enter a number between 1 and 1000 for the message count value!" << std::endl;
        return 0;
    }

    // Topics
    QVector<QString> topicTypes;
    QVector<QString> topicNames;
    QSet<QString> topicNameSet;
    auto areROS2NamesValid = true;
    // Ensure correct topic type and name ordering
    for (auto i = 2; i < arguments.size() - 1; i++) {
        const auto argument = arguments.at(i);

        if (i % 2 == 0) {
            if (!Utils::ROS::isNameROS2Conform(argument) && areROS2NamesValid) {
                const auto errorString = "The topic name does not follow the ROS2 naming convention! More information on ROS2 naming convention is found here:\n"
                                         "https://design.ros2.org/articles/topic_and_service_names.html\n"
                                         "Do you want to continue anyways? [y/n]";
                if (!Utils::CLI::shouldContinue(errorString)) {
                    return 0;
                }
                areROS2NamesValid = false;
            }
            topicNames.push_back(argument);
            topicNameSet.insert(argument);
        } else {
            if (argument != "String" && argument != "Integer" && argument != "Image" && argument != "PointCloud") {
                std::cerr << "The topic type must be either 'String', 'Integer', 'Image' or 'PointCloud'!" << std::endl;
                return 0;
            }
            topicTypes.push_back(argument);
        }
    }

    if (topicTypes.size() != topicNames.size()) {
        std::cerr << "Topic type and topic name size do not match. Please make sure to enter a name for each topic!" << std::endl;
        return 0;
    }
    if (topicNameSet.size() != topicNames.size()) {
        std::cerr << "Duplicate topic names detected. Please make sure that every topic name is unique!" << std::endl;
        return 0;
    }

    // Create thread parameters
    QVector<Parameters::DummyBagParameters::DummyBagTopic> topics;
    for (auto i = 0; i < topicTypes.size(); i++) {
        parameters.topics.push_back({ topicTypes.at(i), topicNames.at(i) });
    }

    if (std::filesystem::exists(parameters.sourceDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The dummy bag file already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const dummyBagThread = new DummyBagThread(parameters);

    QObject::connect(dummyBagThread, &DummyBagThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(dummyBagThread, &DummyBagThread::finished, [] {
        // This signal is thrown even if SIGINT is called, but we haven't finished, only interrupted
        if (signalStatus != SIGINT) {
            std::cout << "" << std::endl; // Extra line to stop flushing
            std::cout << "Creating bag finished!" << std::endl;
        }
        return EXIT_SUCCESS;
    });
    QObject::connect(dummyBagThread, &DummyBagThread::finished, dummyBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Creating dummy bag. Please wait..." << std::endl;
    Utils::CLI::runThread(dummyBagThread, signalStatus);

    return EXIT_SUCCESS;
}
