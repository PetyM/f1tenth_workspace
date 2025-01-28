#ifndef MAPEVALUATINGAGENTBASE_H
#define MAPEVALUATINGAGENTBASE_H

#include "cppagent/agentbase.h"
#include "cppagent/lockedvalue.h"

#include <vector>

class MapEvaluatingAgentBase : public AgentBase
{
    using BaseClass = AgentBase;
public:
    MapEvaluatingAgentBase();

protected:
    using Costmap = std::vector<std::vector<double>>;

    Costmap costmap() const;

private:

    LockedValue<Costmap> m_costmap;

};

#endif // MAPEVALUATINGAGENTBASE_H