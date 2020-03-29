#include "dcel.h"

#include <unordered_set>
#include <sstream>

namespace ads
{
namespace ccpp
{
namespace dcel
{
std::pair<bool, std::string> is_valid(const DoublyConnectedEdgeList& dcel)
{
    std::stringstream errMsg;

    std::unordered_set<const dcel::region_t*> regionSet;
    std::unordered_set<const dcel::half_edge_t*> edgeSet;
    std::unordered_set<const dcel::vertex_t*> vertexSet;

    for (size_t i = 0; i < dcel.regions.size(); i++)
    {
        const auto& region = dcel.regions[i];

        if (!region)
            return {false, (errMsg << "region " << i << " is null", errMsg.str())};

        if (!region->edge)
            return {false, (errMsg << "region " << i << " has null edge", errMsg.str())};

        regionSet.insert(region.get());
    }

    for (size_t i = 0; i < dcel.edges.size(); i++)
    {
        const auto& edge = dcel.edges[i];

        if (!edge)
            return {false, (errMsg << "edge " << i << " is null", errMsg.str())};

        if (!edge->next || !edge->prev)
            return {false, (errMsg << "edge " << i << " has null neighbor", errMsg.str())};

        if (!edge->origin)
            return {false, (errMsg << "edge " << i << " has null origin", errMsg.str())};

        if (!edge->region)
            return {false, (errMsg << "edge " << i << " has null region", errMsg.str())};

        edgeSet.insert(edge.get());
    }

    for (size_t i = 0; i < dcel.vertices.size(); i++)
    {
        const auto& vertex = dcel.vertices[i];

        if (!vertex)
            return {false, (errMsg << "vertex " << i << " is null", errMsg.str())};

        if (!vertex->edge)
            return {false, (errMsg << "vertex " << i << " has null edge", errMsg.str())};

        if (edgeSet.count(vertex->edge) == 0)
            return {false, (errMsg << "vertex " << i << " points to unknown edge", errMsg.str())};

        vertexSet.insert(vertex.get());
    }

    for (size_t i = 0; i < dcel.edges.size(); i++)
    {
        const auto& edge = dcel.edges[i];

        if (edgeSet.count(edge->next) == 0 || edgeSet.count(edge->prev) == 0)
            return {false, (errMsg << "edge " << i << " has unknown neighbor", errMsg.str())};

        if (edge->twin)
        {
            if (edgeSet.count(edge->twin) == 0)
                return {false, (errMsg << "edge " << i << " has unknown twin", errMsg.str())};

            if (edge->twin->twin != edge.get())
                return {false, (errMsg << "edge " << i << " twin does not have edge " << i << " as twin", errMsg.str())};
        }

        if (vertexSet.count(edge->origin) == 0)
            return {false, (errMsg << "edge " << i << " has unknown origin", errMsg.str())};

        if (regionSet.count(edge->region) == 0)
            return {false, (errMsg << "edge " << i << " has unknown region", errMsg.str())};

        if (edge->region != edge->next->region || edge->region != edge->prev->region)
            return {false, (errMsg << "edge " << i << " does not share region with neighbor", errMsg.str())};

        if (edge->next->prev != edge.get() || edge->prev->next != edge.get())
            return {false, (errMsg << "edge " << i << " is not properly linked to neighbors", errMsg.str())};

        if (edge->next == edge.get() || edge->prev == edge.get())
            return {false, (errMsg << "edge " << i << " is neighbor to itself", errMsg.str())};

        if (edge->next == edge->prev)
            return {false, (errMsg << "edge " << i << " has same next and previous", errMsg.str())};
    }

    for (size_t i = 0; i < dcel.regions.size(); i++)
    {
        const auto& region = dcel.regions[i];

        if (edgeSet.count(region->edge) == 0)
            return {false, (errMsg << "region " << i << " points to unknown edge", errMsg.str())};

        if (region->edge->region != region.get())
            return {false, (errMsg << "region " << i << " points to edge that does not point back", errMsg.str())};
    }

    return {true, ""};
}
}
}
}
