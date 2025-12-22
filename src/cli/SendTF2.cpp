#include "SendTF2Thread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_send_tf2 path/to/pcds/dir path/to/bag\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-t or --translation: Translation. Must be in format \"value,value,value\", default is \"0.0,0.0,0.0\".\n";
    std::cout << "-ro or --rotation: Rotation. Must be in format \"value,value,value,value\", default is \"0.0,0.0,0.0,1.0\".\n\n";
    std::cout << "-c or --child: Child frame name.\n";
    std::cout << "-r or --rate: Rate (only for nonstatic transformations). Must be between 1 and 100. If this is not specified, the transformation is static.\n\n";
    std::cout << "-h or --help: Show this help.\n";
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Initialize ROS and Qt
    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (arguments.size() < 1 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    const QStringList checkList{ "-t", "-ro", "-c", "-r", "--translation", "--rotation", "--child", "--rate" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::SendTF2Parameters parameters;
    // A char array means something like 0.12345. We have to convert this to a double value
    const auto charArrayToDouble = [arguments] (const std::string& transformationType, int argumentsIndex, int expectedArraySize) {
        std::stringstream ss(arguments.at(argumentsIndex).toStdString());
        std::string temp;
        std::vector<double> doubleValues;
        auto loopCounter = 0;

        while (getline(ss, temp, ',')) {
            loopCounter++;

            try {
                doubleValues.emplace_back(std::stod(temp));
            } catch (...) {
                throw std::runtime_error("Invalid arguments for " + transformationType + "!");
            }
        }
        // xyz for translations and xyzw for rotation
        if (loopCounter != expectedArraySize) {
            throw std::runtime_error("Please insert the correct number of values for " + transformationType +
                                     " (expected " + std::to_string(expectedArraySize) + ")!");
        }

        return doubleValues;
    };

    // Translation
    if (Utils::CLI::containsArguments(arguments, "-t", "--translation")) {
        const auto translationIndex = Utils::CLI::getArgumentsIndex(arguments, "-t", "--translation") + 1;
        const auto& translationValues = charArrayToDouble("translation", translationIndex, 3);

        parameters.translation[0] = translationValues.at(0);
        parameters.translation[1] = translationValues.at(1);
        parameters.translation[2] = translationValues.at(2);
    }
    // Rotation
    if (Utils::CLI::containsArguments(arguments, "-ro", "--rotation")) {
        const auto rotationIndex = Utils::CLI::getArgumentsIndex(arguments, "-ro", "--rotation") + 1;
        const auto& rotationValues = charArrayToDouble("rotation", rotationIndex, 4);

        parameters.rotation[0] = rotationValues.at(0);
        parameters.rotation[1] = rotationValues.at(1);
        parameters.rotation[2] = rotationValues.at(2);
        parameters.rotation[3] = rotationValues.at(3);
    }

    // Check for optional arguments
    // Child frame name
    if (Utils::CLI::containsArguments(arguments, "-c", "--child")) {
        parameters.childFrameName = arguments.at(Utils::CLI::getArgumentsIndex(arguments, "-c", "--child") + 1);
        std::cout << "NAME " << parameters.childFrameName.toStdString() << "\n";
    }

    auto nodeWrapper = std::make_shared<NodeWrapper>("ros2_utils_tool_tf_node");
    if (!Utils::CLI::containsArguments(arguments, "-r", "--rate")) {
        Utils::ROS::sendStaticTransformation(parameters.translation, parameters.rotation, nodeWrapper);
        std::cout << "TF sent!\n";
        return EXIT_SUCCESS;
    }

    // Rate
    parameters.isStatic = false;
    if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", parameters.rate, 1, 100)) {
        throw std::runtime_error("Please enter a rate in the range of 1 to 100!");
    }

    // Create thread and connect to its informations
    auto* const sendTF2Thread = new SendTF2Thread(parameters);

    QObject::connect(sendTF2Thread, &SendTF2Thread::finished, sendTF2Thread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Sending transformation...\n";
    Utils::CLI::runThread(sendTF2Thread, signalStatus);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
