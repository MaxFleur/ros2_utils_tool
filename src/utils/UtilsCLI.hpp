#pragma once

#include <QString>
#include <QStringList>
#include <QThread>

#include <optional>
#include <string>

#include <signal.h>

// Util functions for the cli tools
namespace Utils::CLI
{
// Determines if an argument list contains invalid arguments
// by comparing it with a matching checklist, returns the invalid argument if found
std::optional<std::string>
containsInvalidParameters(const QStringList& argumentsList,
                          const QStringList& checkList);

// Checks if the command arguments stringlist contains either a specific
// short or long argument and returns the index
bool
containsArguments(const QStringList& argumentsList,
                  const QString&     shortArg,
                  const QString&     longArg);

// Get the index of arguments in a stringlist
int
getArgumentsIndex(const QStringList& argumentsList,
                  const QString&     shortArg,
                  const QString&     longArg);

// Check if an argument to be set with a value is correctly called
bool
checkArgumentValidity(const QStringList& argumentsList,
                      const QString&     shortArg,
                      const QString&     longArg,
                      int&               parameter,
                      int                lowerRange,
                      int                higherRange,
                      // In most cases, the value to set the argument comes directly at the position after it
                      int                argumentListOffset = 1);

// Checks if the topic name is at a valid position in the args list
void
checkTopicParameterPosition(const QStringList& argumentsList);

// If a topic name is existent and the corresponding topic in the according format
void
checkTopicNameValidity(const QStringList& argumentsList,
                       const QString&     bagDirectory,
                       const QString&     topicType,
                       QString&           topicNameToSet);

// Checks if a source bag directory exists and contains a valid bag file
void
checkBagSourceDirectory(const QString& bagDirectory);

// Checks if the parent dir of a specified directory exists
void
checkParentDirectory(const QString& directory,
                     bool           isTarget = true);

// Checks for existing topics in a bag directory and writes the first found topic to a parameter topic name, if found
void
checkForTargetTopic(const QString& directory,
                    QString&       parameterTopicName,
                    const QString& topicType);

// Ask if the tool should continue for cases of invalidacies
bool
shouldContinue(const std::string& message);

// If we should continue with an invalid ROS2 name
bool
continueWithInvalidROS2Name(const QStringList& arguments,
                            QString&           parameterTopicName);

// If we should continue with an already existing target and/or low diskspace
bool
continueExistingTargetLowDiskSpace(const QStringList& arguments,
                                   const QString&     directory);

// Draws a small progress string in the following format:
// ############################--------------------
// 50 charactes, # shows the progress
[[nodiscard]] std::string
drawProgressString(int progress);

// Shows a processing string while something is processed in the background
void
showProcessingString(bool& isProcessing,
                     int   toolOperation);

// Run the thread handling the main operation
void
runThread(QThread*               thread,
          volatile sig_atomic_t& signalStatus);

static constexpr int CLI_COMPRESS = 0;
static constexpr int CLI_DECOMPRESS = 1;
static constexpr int CLI_MERGE = 2;
}
