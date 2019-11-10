// Datastructures.cc

#include "datastructures.hh"

#include <random>
#include <algorithm>
#include <cmath>

std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end)
{
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

// Template for updating a key in a map determined by iter with newKey.
template <typename T>
void updateKey(T &map, typename T::key_type newKey, typename T::iterator iter)
{
    typename T::node_type nodeHandle = map.extract(iter);
    nodeHandle.key() = newKey;
    map.insert(std::move(nodeHandle));
}

// Returns a vector of map's values
template <typename M>
std::vector<typename M::mapped_type> mapValsToVec(const M &map)
{
    std::vector< typename M::mapped_type > toVec;
    toVec.reserve(map.size());

    for (const typename M::value_type &pair : map)
    {
        toVec.push_back(pair.second);
    }
    return toVec;
}

// Returns a vector of map's keys
template <typename M>
std::vector<typename M::key_type> mapKeysToVec(const M &map)
{
    std::vector<typename M::key_type> toVec;
    toVec.reserve(map.size());

    for (const typename M::value_type &pair : map)
    {
        toVec.push_back(pair.first);
    }
    return toVec;
}


// Default constructor initialises the search flags as false.
Datastructures::Datastructures() : _dijkstraValid(false), _bfsValid(false) {}
Datastructures::~Datastructures() {}

int Datastructures::beacon_count()
{
    return static_cast<int>(_beacons.size());
}

void Datastructures::clear_beacons()
{
    _beacons.clear();
    _sortedNames.clear();
    _sortedBrightness.clear();
}

std::vector<BeaconID> Datastructures::all_beacons()
{
    return mapKeysToVec(_beacons);
}

bool Datastructures::add_beacon(BeaconID id, const std::string& name,
                                Coord xy, Color color)
{
    if (_beacons.find(id) != _beacons.end())
    {
        return false;
    }
    int brightness = color.brightness();
    sNameIter nameIter =_sortedNames.insert({name, id});
    sColorIter colorIter =_sortedBrightness.insert({brightness, id});

    Beacon beacon(name, xy, color, nameIter, colorIter);
    _beacons.insert({id, beacon});

    return true;
}

std::string Datastructures::get_name(BeaconID id)
{
    const auto iter = _beacons.find(id);

    if (iter == _beacons.end())
    {
        return NO_NAME;
    }
    return iter->second.name;
}

Coord Datastructures::get_coordinates(BeaconID id)
{
    const auto iter = _beacons.find(id);

    if (iter == _beacons.end())
    {
        return NO_COORD;
    }
    return iter->second.xy;
}

Color Datastructures::get_color(BeaconID id)
{
    const auto iter = _beacons.find(id);

    if (iter == _beacons.end())
    {
        return NO_COLOR;
    }
    return iter->second.color;
}


std::vector<BeaconID> Datastructures::beacons_alphabetically()
{
    return mapValsToVec(_sortedNames);
}

std::vector<BeaconID> Datastructures::beacons_brightness_increasing()
{
    return mapValsToVec(_sortedBrightness);
}

BeaconID Datastructures::min_brightness()
{
    return minOrMaxBrightness(false);
}

BeaconID Datastructures::max_brightness()
{
   return minOrMaxBrightness(true);
}

BeaconID Datastructures::minOrMaxBrightness(const bool max)
{
    if (_sortedBrightness.empty())
    {
        return NO_ID;
    }
    else if (max)
    {
        return _sortedBrightness.rbegin()->second;
    }
    else
    {
        return _sortedBrightness.begin()->second;
    }
}

std::vector<BeaconID> Datastructures::find_beacons(std::string const& name)
{
    auto iterPair = _sortedNames.equal_range(name);
    std::vector<BeaconID> beaconIDs;

    // If the iterators point to the same place, nothing was found.
    if (iterPair.first == iterPair.second)
    {
        return beaconIDs;
    }
    beaconIDs.reserve(static_cast<unsigned>(
                        std::distance(iterPair.first, iterPair.second)));

    // Add all elements from the equal_range to the vector
    std::for_each(iterPair.first, iterPair.second,
                  [&beaconIDs](const auto &pair)
    {
        beaconIDs.push_back(pair.second);
    });
    std::sort(beaconIDs.begin(), beaconIDs.end());

    return beaconIDs;
}

bool Datastructures::change_beacon_name(BeaconID id, const std::string& newname)
{
    auto iter = _beacons.find(id);

    if (iter == _beacons.end())
    {
        return false;
    }
    const sNameIter nameIter = iter->second.namesIter;

    // Update _beacons and _sortedNames
    iter->second.name = newname;
    updateKey(_sortedNames, newname, nameIter);
    return true;
}

bool Datastructures::change_beacon_color(BeaconID id, Color newcolor)
{
    auto iter = _beacons.find(id);

    if (iter == _beacons.end())
    {
        return false;
    }
    const sColorIter colorIter = iter->second.colorsIter;

    // Update _beacons and _sortedBrightness
    iter->second.color = newcolor;
    updateKey(_sortedBrightness, newcolor.brightness(), colorIter);
    return true;
}

bool Datastructures::add_lightbeam(BeaconID sourceid, BeaconID targetid)
{
    const auto sIter = _beacons.find(sourceid);
    const auto tIter = _beacons.find(targetid);
    const auto bEnd = _beacons.end();

    // If sourceid or targetid were not found or if sourceid's Beacon already
    // points to a target -> return false
    if ( (sIter == bEnd || tIter == bEnd) && sIter->second.targetID != NO_ID)
    {
        return false;
    }
    // Add to source's lightbeam, target's sources
    sIter->second.targetID = targetid;
    tIter->second.sources.insert(sourceid);
    return true;
}

std::vector<BeaconID> Datastructures::get_lightsources(BeaconID id)
{
    auto scIter = _beacons.find(id);
    if (_beacons.find(id) == _beacons.end())
    {
        return {NO_ID};
    }
    const std::set<BeaconID> &sources = scIter->second.sources;
    std::vector<BeaconID> lightsources(sources.begin(), sources.end());

    return lightsources;
}

std::vector<BeaconID> Datastructures::path_outbeam(BeaconID id)
{
    std::vector<BeaconID> outbeam;
    const auto iter = _beacons.find(id);

    if (iter == _beacons.end())
    {
        return {NO_ID};
    }
    outbeam.push_back(id);
    BeaconID lightbeam = iter->second.targetID;

    while (lightbeam != NO_ID)
    {
        outbeam.push_back(lightbeam);
        lightbeam = _beacons.find(lightbeam)->second.targetID;
    }
    return outbeam;
}

bool Datastructures::remove_beacon(BeaconID id)
{
    auto iter = _beacons.find(id);
    const BeaconID &targetID = iter->second.targetID;

    if (iter == _beacons.end())
    {
        return false;
    }

    // Remove from Beacons pointing to this ID (sources)
    for (const BeaconID &sourceID : iter->second.sources)
    {
        _beacons.find(sourceID)->second.targetID = NO_ID;
    }

    if (targetID != NO_ID)
    {
        // Erase from targetID Beacon's sources
        auto targetIter = _beacons.find(targetID);
        targetIter->second.sources.erase(id);
    }

    // Get the iterators for the multimaps and erase from beacons
    sNameIter nameIter = iter->second.namesIter;
    sColorIter colorIter = iter->second.colorsIter;
    _beacons.erase(iter);

    // Erase from multimaps
    _sortedNames.erase(nameIter);
    _sortedBrightness.erase(colorIter);

    return true;
}

Datastructures::pIntID Datastructures::getDeepestLeaf(const beaconIter bIter,
                                                      pIntID deepLeaf)
{
    const Beacon &beacon = bIter->second;
    pIntID hIdPair = {0, bIter->first};
    bool newDeepest = false;

    if (beacon.sources.empty()) {
        return hIdPair;
    }

    for (const BeaconID &sourceID : beacon.sources)
    {
       hIdPair = getDeepestLeaf(_beacons.find(sourceID));

       // If the depth reached is more than with the previous deepLeaf
       if (hIdPair.first > deepLeaf.first)
       {
           deepLeaf = hIdPair;
           newDeepest = true;
       }
    }
    if (!newDeepest)
    {
        hIdPair.first += 1;
        return hIdPair;
    }
    else {
        deepLeaf.first += 1;
        return deepLeaf;
    }
}

std::vector<BeaconID> Datastructures::path_inbeam_longest(BeaconID id)
{
    auto iter = _beacons.find(id);
    if (iter == _beacons.end())
    {
        return {NO_ID};
    }
    pIntID leaf = getDeepestLeaf(iter);

    // If the id's Beacon is a leaf
    if (leaf.first == 0)
    {
        return {id};
    }
    std::vector<BeaconID> path;
    path.reserve(static_cast<size_t>(leaf.first));
    path.push_back(leaf.second);
    BeaconID targetID = _beacons.find(leaf.second)->second.targetID;

    while (targetID != id)
    {
        path.push_back(targetID);
        targetID = _beacons.find(targetID)->second.targetID;
    }
    path.push_back(id);
    return path;
}

Color Datastructures::totalColorRecursion(const beaconIter bIter)
{
    Beacon beacon = bIter->second;
    Color colorSum(beacon.color);

    // If nothing is pointing to this Beacon the color is the color the beacon.
    if (beacon.sources.empty()) {
        return colorSum;
    }
    // Calculate the arithmetical average of the rgb components given
    // the amount of sources.
    int denom = static_cast<int>(beacon.sources.size()) + 1;

    for (const BeaconID &sourceID : beacon.sources)
    {
        colorSum += totalColorRecursion(_beacons.find(sourceID));
    }
    Color totalColor = colorSum/denom;

    return totalColor;
}

Color Datastructures::total_color(BeaconID id)
{
    auto iter = _beacons.find(id);
    if (iter == _beacons.end())
    {
        return NO_COLOR;
    }
    return totalColorRecursion(iter);
}

std::vector<Coord> Datastructures::all_xpoints()
{
    std::vector<Coord> coords;
    coords.reserve(_xPoints.size());

    for (const auto &pair : _xPoints)
    {
       coords.push_back(pair.first);
    }
    std::sort(coords.begin(), coords.end());

    return coords;
}

std::pair<bool, Datastructures::fibreIter>
    Datastructures::fibresFind(CoordRefPair &cPair)
{
    CoordRefPair opposite = {cPair.second, cPair.first};

    if (opposite < cPair)
    {
        // Smaller coord first, swap the coordinates. Uses Coord's implicit
        // move constructor
        std::swap(cPair.first, cPair.second);
    }

    fibreIter fibreIter = _allFibres.find(cPair);
    auto end = _allFibres.end();

    if (fibreIter == _allFibres.end())
    {
        return {false, end};
    }

    return {true, fibreIter};
}

bool Datastructures::add_fibre(Coord xpoint1, Coord xpoint2, Cost cost)
{
    CoordRefPair cPair = {xpoint1, xpoint2};

    if (xpoint1 == xpoint2)
    {
        return false;
    }
    else if (fibresFind(cPair).first)
    {
        return false;
    }

    return addFibre(xpoint1, xpoint2, cost);
}

bool Datastructures::addFibre(const Coord& from, const Coord &to, const int cost)
{
    auto xStartIter = _xPoints.find(from);
    auto xEndIter = _xPoints.find(to);
    bool hasStart = xStartIter != _xPoints.end();
    bool hasEnd = xEndIter != _xPoints.end();

    Fibre fibre(cost);
    XPoint* xStart;
    XPoint* xEnd;

    // Logic for creating or fetching the corresponding XPoint for the coords.
    if (hasStart && hasEnd)
    {
        xStart = &(xStartIter->second);
        xEnd = &(xEndIter->second);
    }
    else if (hasStart)
    {
        xStart = &(xStartIter->second);
        xEnd = &(_xPoints.emplace(to, to).first->second);
    }
    else if (hasEnd)
    {
        xStart = &(_xPoints.emplace(from, from).first->second);
        xEnd = &(xEndIter->second);
    }
    else {
        xStart = &(_xPoints.emplace(from, from).first->second);
        xEnd = &(_xPoints.emplace(to, to).first->second);
    }

    fibre.start = xStart;
    fibre.end = xEnd;

    auto fibreIter = _allFibres.insert(fibre).first;
    const Fibre* fibrePtr = &(*fibreIter);

    // Add to start and end XPoint's fibres
    xStart->fibres.insert(fibrePtr);
    xEnd->fibres.insert(fibrePtr);

    // Update the algorithm flags
    _dijkstraValid = false;
    _bfsValid = false;

    return true;
}

std::vector<std::pair<Coord, Cost>> Datastructures::get_fibres_from(Coord xpoint)
{
    std::vector<std::pair<Coord, Cost>> coordCosts;
    auto iter = _xPoints.find(xpoint);

    if (iter == _xPoints.end())
    {
        return coordCosts;
    }
    const auto &fibres = iter->second.fibres;
    coordCosts.reserve(fibres.size());

    for (const Fibre* fibre : fibres)
    {
        if (fibre->start->coord == xpoint)
        {
            coordCosts.emplace_back(fibre->end->coord, fibre->cost);
        }
        else {
            coordCosts.emplace_back(fibre->start->coord, fibre->cost);
        }
    }
    return coordCosts;
}

std::vector<std::pair<Coord, Coord>> Datastructures::all_fibres()
{
    std::vector<std::pair<Coord, Coord>> startEnds;
    startEnds.reserve(_allFibres.size());

    for (const Fibre &fibre : _allFibres)
    {
        startEnds.emplace_back(fibre.start->coord, fibre.end->coord);
    }
    return startEnds;
}

bool Datastructures::remove_fibre(Coord xpoint1, Coord xpoint2)
{
    CoordRefPair cPair = {xpoint1, xpoint2};
    auto [fibreFound, fibreIter] = fibresFind(cPair);

    if (!fibreFound)
    {
        return false;
    }

    // Fetch the both XPoints and their start-outgoing and end-incoming.
    auto xStartIter = _xPoints.find(xpoint1);
    auto xEndIter = _xPoints.find(xpoint2);
    auto &sFibres = xStartIter->second.fibres;
    auto &eFibres = xEndIter->second.fibres;

    // Find the iterators to the Fibre-ptrs.
    auto sIter = sFibres.find(cPair);
    auto eIter = eFibres.find(cPair);

    // Erase from start-outgoing and end-incoming
    sFibres.erase(sIter);
    eFibres.erase(eIter);

    _allFibres.erase(fibreIter);

    // If XPoint's fibres set was left empty. Erase the XPoint.
    if (sFibres.empty())
    {
        _xPoints.erase(xStartIter);
    }
    if (eFibres.empty())
    {
        _xPoints.erase(xEndIter);
    }

    // Update the algorithm flags
    _dijkstraValid = false;
    _bfsValid = false;

    return true;
}

void Datastructures::clear_fibres()
{
    _allFibres.clear();
    _xPoints.clear();
}

void Datastructures::resetDFS(const std::vector<XPoint*> &xPtrs)
{
    for (XPoint* xPtr : xPtrs)
    {
        xPtr->color = White;
        xPtr->hereFrom = nullptr;
    }
}

void Datastructures::resetColorsAndUFibres()
{
    for (auto &pair : _xPoints)
    {
        pair.second.color = White;
        pair.second.usedFibre = nullptr;
    }
}

XPoint *Datastructures::getAdjacent(const Fibre *fibre, const XPoint *from)
{
    // When going backwards adjacent is the start
    if (fibre->end == from)
    {
        return fibre->start;
    }
    else {
        return fibre->end;
    }

}

std::vector<std::pair<Coord, Cost>> Datastructures::route_any(Coord fromxpoint,
                                                              Coord toxpoint)
{
    // Check for search validity in-order: BFS -> Dijkstra and call
    // the right function accordingly. If neither are valid call BFS.

    bool useBfs = true;

    if (_bfsValid);
    else if (_dijkstraValid) useBfs = false;

    return routeFastOrShort(fromxpoint, toxpoint, useBfs);
}


void Datastructures::doBFS(XPoint* xPtr)
{
    // Use deque for queue implementation
    std::deque<XPoint*> deque;

    XPoint* start = xPtr;
    start->color = Gray;
    start->dBfs= 0;

    deque.push_back(start);

    while (!deque.empty())
    {
        XPoint* node = deque.front();
        deque.pop_front();

        // Push white-colored adjecent XPoints to deque and resetColorsOf
        for (const Fibre* fibre : node ->fibres)
        {
            XPoint* adjacent = getAdjacent(fibre, node);

            if (adjacent->color == White)
            {
                adjacent->color = Gray;
                adjacent->dBfs = node ->dBfs + fibre->cost;
                adjacent->piBfs = node;

                deque.push_back(adjacent);
            }
        }
        node ->color = Black;
    }
}


bool Datastructures::routeFromTo(const XPoint* fromPtr,
                                 const XPoint* toPtr,
                                 std::vector<std::pair<Coord, Cost>> &route,
                                 const bool bfs)
{
    const XPoint* pi;
    Cost d;

    // Determine if we are routing shortest or fastest
    if (bfs)
    {
        pi = toPtr->piBfs;
        d = toPtr->dBfs;
    }
    else {
        pi = toPtr->piDij;
        d = toPtr->dDij;
    }

    // Routing logic
    if (fromPtr == toPtr)
    {
        route.emplace_back(fromPtr->coord, d);
        return true;
    }
    else if (!pi)
    {
        return false;
    }
    else {
        if (!routeFromTo(fromPtr, pi, route, bfs))
        {
            return false;
        }
        route.emplace_back(toPtr->coord, d);
        return true;
    }
}

std::pair<XPoint*, XPoint*>
    Datastructures::initRouteSearch(const Coord &from, const Coord &to)
{
    auto xStartIter = _xPoints.find(from);
    auto xEndIter = _xPoints.find(to);
    auto end = _xPoints.end();

    if (xStartIter == end || xEndIter == end)
    {
        return {nullptr, nullptr};
    }

    XPoint* startPtr = &xStartIter->second;
    XPoint* endPtr = &xEndIter->second;

    return {startPtr, endPtr};
}

void Datastructures::resetBFSorDij(const bool bfs)
{
    for (auto &pair : _xPoints)
    {
        if (bfs)
        {
            pair.second.dBfs = INT_MAX;
            pair.second.piBfs = nullptr;
        }
        else {
            pair.second.dDij = INT_MAX;
            pair.second.piDij = nullptr;
        }
    }
}

std::vector<std::pair<Coord, Cost>>
    Datastructures::routeFastOrShort(const Coord &from,
                                     const Coord &to,
                                     const bool bfs)
{
    std::vector<std::pair<Coord, Cost>> coordCosts;

    auto [startPtr, endPtr] = initRouteSearch(from, to);

    // If one or both of the coordinates were invalid
    if (!startPtr || !endPtr)
    {
        return coordCosts;
    }

    void (Datastructures::*searchAlg)(XPoint* start) = nullptr;

    // If not valid. Call the right algorithm
    if (bfs && !_bfsValid)
    {
        searchAlg = &Datastructures::doBFS;
    }
    else if (!bfs && !_dijkstraValid)
    {
        searchAlg = &Datastructures::doDijkstra;
    }
    // If not null. Search has to be performed
    if (searchAlg)
    {
        resetBFSorDij(bfs);
        (this->*searchAlg)(startPtr);

        // Call the searchAlg till every XPoint is black. Otherwise we could
        // possibly not update all graphs.
        for (auto &pair : _xPoints)
        {
            if (pair.second.color != Black)
            {
               (this->*searchAlg)(&pair.second);
            }
        }
        // Reset colors
        resetColorsAndUFibres();

        // Update the flags
        if (bfs)
        {
            _bfsValid = true;
        }
        else {
            _dijkstraValid = true;
        }
    }

    // Route and check if the route was valid.
    if (!routeFromTo(startPtr, endPtr, coordCosts, bfs))
    {
        return {};
    }
    else {
        return coordCosts;
    }
}

void Datastructures::dijkstraRelax(XPoint *check,
                                   const XPoint *min,
                                   const Cost fCost)
{
    const int sum = min->dDij + fCost;

    if (check->dDij > sum)
    {
        check->dDij = sum;
        check->piDij = min;
    }
}

void Datastructures::dijkstraPushAdj(Datastructures::dijPQueue &pQueue,
                                     const XPoint* minNode)
{
    for (const Fibre* fibre : minNode->fibres)
    {
        XPoint* adjacent = getAdjacent(fibre, minNode);

        if (adjacent->color == White)
        {
            adjacent->color = Gray;

            // Relax has to be done before emplace as the inherent
            // order of the priority queue could otherwise turn into a mess.
            dijkstraRelax(adjacent, minNode, fibre->cost);
            pQueue.emplace(adjacent);
        }
        else {
            dijkstraRelax(adjacent, minNode, fibre->cost);
        }
    }
}

void Datastructures::doDijkstra(XPoint* start)
{
    // Min-heap queue with XPoint-ptrs
    std::priority_queue<XPoint*, std::vector<XPoint*>, PQDijComp> pQueue;

    start->color = Gray;
    start->dDij = 0;
    pQueue.emplace(start);

    // Perform dijkstra's
    while (!pQueue.empty())
    {
        XPoint* minNode = pQueue.top();
        pQueue.pop();

        dijkstraPushAdj(pQueue, minNode);
        minNode->color = Black;
    }  
}

std::vector<std::pair<Coord, Cost>>
    Datastructures::route_least_xpoints(Coord fromxpoint, Coord toxpoint)
{
    return routeFastOrShort(fromxpoint, toxpoint, true);
}

std::vector<std::pair<Coord, Cost>>
    Datastructures::route_fastest(Coord fromxpoint, Coord toxpoint)
{
    return routeFastOrShort(fromxpoint, toxpoint, false);
}

std::vector<Coord> Datastructures::doDFSAndReset(XPoint* start)
{
    std::deque<XptrCost> deque;

    // Store the ptrs to the handled XPoint-objects for color and
    // hereFrom-ptr reset after DFS-search.
    std::vector<XPoint*> resetHandled;

    std::vector<Coord> coords;

    resetHandled.emplace_back(start);
    deque.emplace_back(start, 0);

    while (!deque.empty())
    {
        XptrCost popped = deque.back();
        XPoint* node = popped.first;
        deque.pop_back();

        if (node->color == White)
        {
            node->color = Gray;
            deque.push_back(popped);

            // If push returns true, cycle was found.
            if (DFSPushAdjacent(deque, resetHandled))
            {
                copyCycle(deque, coords);
                resetDFS(resetHandled);

                return coords;
            }
        }
        else {
            node->color = Black;
        }
    }
    resetDFS(resetHandled);

    // Return empty coords
    return coords;
}

bool Datastructures::DFSPushAdjacent(std::deque<XptrCost> &deq,
                                     std::vector<XPoint*> &resetColorsOf)
{
    const XptrCost &pair = deq.back();
    XPoint* node = pair.first;

    // Push white-colored adjacent XPoints to deque and resetColorsOf and
    // detect cycles.
    for (const Fibre* fibre : node->fibres)
    {
        XPoint* adjacent = getAdjacent(fibre, node);

        if (node->hereFrom && node->hereFrom == adjacent)
        {
            // Forbid going back
            continue;
        }
        const Cost routeCost = pair.second + fibre->cost;

        if (adjacent->color == White)
        {
            resetColorsOf.emplace_back(adjacent);
            deq.emplace_back(adjacent, routeCost);
        }
        else if (adjacent->color == Gray)
        {
            deq.emplace_back(adjacent, routeCost);

            // Place the new hereFrom XPoint* on top of the deq. Leave the old
            // untouched.
            deq.emplace_back(node, 0);

            return true;
        }

        // Update hereFrom ptr.
        adjacent->hereFrom = node;
    }
    return false;
}

void Datastructures::copyCycle(std::deque<XptrCost> &deq,
                               std::vector<Coord> &toVec) const
{
    // secondLast was pushed to the deq last
    const XPoint* secondLast = deq.back().first;
    deq.pop_back();

    // Get start and end
    const XPoint* node = deq.back().first;
    const XPoint* end = deq.front().first;

    // Next is secondLast
    toVec.push_back(node->coord);
    node = secondLast;

    // Push the route to vector using the hereFrom ptrs
    while (node != end)
    {
        toVec.push_back(node->coord);
        node = node->hereFrom;
    }
    toVec.push_back(end->coord);

    // The path the program wants is in reverse
    std::reverse(toVec.begin(), toVec.end());
}


std::vector<Coord> Datastructures::route_fibre_cycle(Coord startxpoint)
{
    std::vector<Coord> cycle;
    auto startIter =_xPoints.find(startxpoint);

    if (startIter == _xPoints.end())
    {
        return cycle;
    }

    XPoint* start = &(startIter->second);
    return doDFSAndReset(start);
}

void Datastructures::trimPushAdj(trimPQue &pQue,
                                 const XPoint* minNode)
{
    for (const Fibre* fibre : minNode->fibres)
    {
        XPoint* adjacent = getAdjacent(fibre, minNode);

        // Grey means it's been added once
        if (adjacent->color == Gray)
        {
            const Fibre* oldFibre = adjacent->usedFibre;
            if (oldFibre->cost > fibre->cost)
            {
                // new usedFibre is cheaper
                adjacent->usedFibre = fibre;

                // Add the cheaper duplicate to pQue
                pQue.emplace(fibre->cost, adjacent);
            }
        }
        else if (adjacent->color == White)
        {
            adjacent->color = Gray;

            // Stores the corresponding Fibre
            adjacent->usedFibre = fibre;

            pQue.emplace(fibre->cost, adjacent);
        }
        // Black means it's already in MST

    }
}

void Datastructures::MSTPrims(XPoint* start,
                              std::set<Fibre, FibreCompare> &spanningFibres,
                              Cost &allGraphsCost)
{
    using pQuePair = std::pair<Cost, XPoint*>;

    Cost trimCost = 0;

    // Min-heap queue with Cost-XPoint* pairs
    std::priority_queue<pQuePair, std::vector<pQuePair>, PQTrimComp> pQueue;

    pQueue.emplace(0, start);

    // Black is used to determine if XPoint is already in MST
    // (Mininum spanning tree).
    while (!pQueue.empty())
    {
        XPoint* minNode = pQueue.top().second;
        pQueue.pop();

        if (minNode->color != Black)
        {
            minNode->color = Black;
            const Fibre* fPtr = minNode->usedFibre;
            if (fPtr)
            {
                spanningFibres.insert(*fPtr);
                trimCost += fPtr->cost;
            }
        }
        trimPushAdj(pQueue, minNode);
    }
    allGraphsCost += trimCost;
}

void Datastructures::trimTree(const std::set<Fibre, FibreCompare> &inSpanning)
{
    // Perform the trimming operation
    static FibreCompare comp;
    std::vector<Fibre> notInSpanning;
    notInSpanning.reserve(_allFibres.size()- inSpanning.size());

    // Set difference {_allFibres - inSpanning} = {notInSpanning}
    std::set_difference(_allFibres.begin(), _allFibres.end(),
                        inSpanning.begin(), inSpanning.end(),
                        std::back_inserter(notInSpanning), comp);

    // Remove the fibres not in the the tree
    for (const Fibre &fibre : notInSpanning)
    {
        // Remove fibres not in spanning tree.
        remove_fibre(fibre.start->coord, fibre.end->coord);
    }
}


Cost Datastructures::trim_fibre_network()
{
    if (_allFibres.empty())
    {
        return NO_COST;
    }
    Cost allGraphsCost = 0;

    std::set<Fibre, FibreCompare> spanningFibres;
    XPoint* start = &(_xPoints.begin()->second);

    MSTPrims(start, spanningFibres, allGraphsCost);

    // Keep trimming until all XPoints are black
    for (auto &pair : _xPoints)
    {
        if (pair.second.color != Black)
        {
           MSTPrims(&pair.second, spanningFibres, allGraphsCost);
        }
    }
    trimTree(spanningFibres);
    resetColorsAndUFibres();

    return allGraphsCost;
}
