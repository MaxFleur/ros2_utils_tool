#pragma once

#include <QString>
#include <QStringList>
#include <QThread>

#include <string>

#include <signal.h>

// Util functions for the cli tools
namespace Utils::CLI
{
// Determines if an argument list contains invalid arguments
// by comparing it with a matching checklist
bool
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

// Ask if the tool should continue for cases of invalidacies
bool
shouldContinue(const std::string& message);

// Checks if the topic name is at a valid position in the args list
bool
isTopicParameterAtValidPosition(const QStringList& argumentsList);

// If a topic name is existent and the corresponding topic in the according format
bool
isTopicNameValid(const QStringList& argumentsList,
                 const QString&     bagDirectory,
                 const QString&     topicType,
                 QString&           topicNameToSet);

// If we should continue with an invalid ROS2 name
bool
continueWithInvalidROS2Name(const QStringList& argumentsList,
                            QString&           parameterTopicName);

// Draws a small progress string in the following format:
// ############################--------------------
// 50 charactes, # shows the progress
[[nodiscard]] std::string
drawProgressString(int progress);

// Run the thread handling the main operation
void
runThread(QThread*               thread,
          volatile sig_atomic_t& signalStatus);
}
