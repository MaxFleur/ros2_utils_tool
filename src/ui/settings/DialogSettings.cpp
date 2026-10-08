#include "DialogSettings.hpp"

DialogSettings::DialogSettings(Parameters::DialogParameters& parameters, const QString& groupName) :
    GeneralSettings(groupName, false)
{
    registerParameter("max_threads", parameters.maxNumberOfThreads, std::thread::hardware_concurrency());
    registerParameter("low_diskspace_threshold", parameters.lowDiskspaceThreshold, static_cast<unsigned int>(10));
    registerParameter("hw_acc", parameters.useHardwareAcceleration, false);
    registerParameter("save_parameters", parameters.saveParameters, false);
    registerParameter("predefined_topic_names", parameters.usePredefinedTopicNames, true);
    registerParameter("warn_ros2_name_convention", parameters.warnROS2NameConvention, false);
    registerParameter("warn_target_overwrite", parameters.warnTargetOverwrite, true);
    registerParameter("warn_low_disk_space", parameters.warnLowDiskSpace, true);

    read();
}
