#ifndef MAPEVALUATINGAGENTBASE_H
#define MAPEVALUATINGAGENTBASE_H

#include "cppagent/agentbase.h"
#include "cppagent/trajectoryevaluation.h"

#include <opencv2/opencv.hpp>

#include <shared_mutex>
#include <vector>

class MapEvaluatingAgentBase : public AgentBase
{
    using BaseClass = AgentBase;
public:
    MapEvaluatingAgentBase();

protected:
    const std::string& costmapTopic() const;
    const std::string& mapName() const;
    const std::string& mapPath() const;

    using Trajectory = std::vector<State>;
    std::vector<TrajectoryEvaluation> evaluateTrajectories(const std::vector<Trajectory>& trajectories) const;


private:
    cv::Mat m_baseCostmap;
    
    std::string m_costmapTopic;
    std::string m_mapName;
    std::string m_mapPath;
};

#endif // MAPEVALUATINGAGENTBASE_H