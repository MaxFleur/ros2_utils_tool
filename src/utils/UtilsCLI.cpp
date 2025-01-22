#include "UtilsCLI.hpp"

#include <iostream>

namespace Utils::CLI
{
bool
containsArguments(const QStringList& stringList, const QString& shortArg, const QString& longArg)
{
    return stringList.contains(shortArg) || stringList.contains(longArg);
}


int
getArgumentsIndex(const QStringList& stringList, const QString& shortArg, const QString& longArg)
{
    return std::max(stringList.lastIndexOf(shortArg), stringList.lastIndexOf(longArg));
}


bool
checkArgumentValidity(const QStringList& stringList, const QString& shortArg, const QString& longArg, int& parameter, int lowerRange, int higherRange)
{
    if (!containsArguments(stringList, shortArg, longArg)) {
        return true;
    }

    const auto argumentIndex = std::max(stringList.lastIndexOf(shortArg), stringList.lastIndexOf(longArg));
    if (stringList.at(argumentIndex) == stringList.last()) {
        return false;
    }
    if (parameter = stringList.at(argumentIndex + 1).toInt(); parameter < lowerRange || parameter > higherRange) {
        return false;
    }

    return true;
}


bool
shouldContinue(const std::string& message)
{
    std::string input;

    while (true) {
        std::cout << message << std::endl;
        std::cin >> input;

        if (input == "y") {
            return true;
        } else if (input == "n") {
            return false;
        }
    }
}


std::string
drawProgressString(int progress)
{
    const int numberOfHashtags = ((float) progress / 100.0f) * 50;
    const auto numberOfDashes = 50 - numberOfHashtags;

    const auto progressString = std::string(numberOfHashtags, '#') + std::string(numberOfDashes, '-');
    return progressString;
}
}
