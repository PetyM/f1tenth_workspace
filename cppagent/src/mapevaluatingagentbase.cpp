#include "cppagent/mapevaluatingagentbase.h"

#include <execution>
#include <filesystem>


MapEvaluatingAgentBase::MapEvaluatingAgentBase()
    : BaseClass()
    , m_baseCostmap()
{
    declare_parameter("costmap_topic", "costmap");
    m_costmapTopic = get_parameter("costmap_topic").as_string();

    declare_parameter("map_name", "");
    m_mapName = get_parameter("map_name").as_string();

    declare_parameter("map_folder_path", "");
    m_mapPath = get_parameter("map_folder_path").as_string();

    std::string costmapPath = std::format("{}_costmap.png", m_mapName);
    if (std::filesystem::exists(costmapPath))
    {
        m_baseCostmap = cv::imread(costmapPath);
    }
    else
    {
        std::string mapPath = std::format("{}/{}", m_mapPath, m_mapName);
        cv::Mat map = cv::imread(mapPath);
    }
}

const std::string& MapEvaluatingAgentBase::costmapTopic() const
{
    return m_costmapTopic;
}

const std::string& MapEvaluatingAgentBase::mapName() const
{
    return m_mapName;
}

const std::string& MapEvaluatingAgentBase::mapPath() const
{
    return m_mapPath;
}

std::vector<TrajectoryEvaluation> MapEvaluatingAgentBase::evaluateTrajectories(const std::vector<MapEvaluatingAgentBase::Trajectory>& trajectories) const
{
    std::vector<TrajectoryEvaluation> result = {};

    const auto evaluateTrajectory = [](const Trajectory& trajectory) -> TrajectoryEvaluation
    {
        return {};
    };

    std::transform(std::execution::par_unseq, trajectories.begin(), trajectories.end(), result.begin(), evaluateTrajectory);
    return result;
}
