// Datastructures.hh

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include "structs.hh"

#include <climits>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <deque>
#include <queue>


class Datastructures
{
public:
    // The datastructures' default constructors are called by the default
    // constructor
    Datastructures();

    // The datastructures' destructors are called automatically by the default
    // destructor
    ~Datastructures();

    // Estimate of performance: O(1)
    // Short rationale for estimate: std::unordered_map::size cplusplus.com
    int beacon_count();

    // Estimate of performance: O(n) where n is the size of the data
    // structures (same for all)
    // Short rationale for estimate: std::unordered_map::clear cplusplus.com and
    // std::multimap::clear cplusplus.com
    void clear_beacons();

    // Estimate of performance: O(n)
    // Short rationale for estimate: push_back operation is done n times without
    // reallocation in between as the space is reserved beforehand.
    std::vector<BeaconID> all_beacons();

    // Estimate of performance: O(log n) where n is size of all the maps
    // Short rationale for estimate: multimap::insert cplusplus.com and as
    // std::unordered_map::insert is amortized constant.
    bool add_beacon(BeaconID id, std::string const& name, Coord xy, Color color);

    // Estimate of performance: O(1)
    // Short rationale for estimate: unordered_map::find cplusplus.com
    std::string get_name(BeaconID id);

    // Estimate of performance: O(1)
    // Short rationale for estimate: unordered_map::find cplusplus.com
    Coord get_coordinates(BeaconID id);

    // Estimate of performance: O(1)
    // Short rationale for estimate: unordered_map::find cplusplus.com
    Color get_color(BeaconID id);

    // Estimate of performance: O(n) where n is _sortedNames.size
    // Short rationale for estimate: push_back operation is done n times without
    // reallocation in between as the space is reserved beforehand.
    std::vector<BeaconID> beacons_alphabetically();

    // Estimate of performance: O(n) where n is _sortedColors.size
    // Short rationale for estimate: push_back operation is done n times without
    // reallocation in between as the space is reserved beforehand.
    std::vector<BeaconID> beacons_brightness_increasing();

    // Estimate of performance: O(1)
    // Short rationale for estimate: multimap::begin cplusplus.com
    // calls minOrMaxBrightness
    BeaconID min_brightness();

    // Estimate of performance: O(1)
    // Short rationale for estimate: multimap::rbegin cplusplus.com
    // calls minOrMaxBrightness
    BeaconID max_brightness();

    // Estimate of performance: O(log m + n log n) where m is _sortedNames.size.
    // and n is the distance between eq_range first and last (small)
    // Short rationale for estimate: std::multimap::equal_range cppreference.com
    // and std::sort cppreference.com
    std::vector<BeaconID> find_beacons(std::string const& name);

    // Estimate of performance: O(1)
    // Short rationale for estimate: The iterator for this id in _sortedNames
    // is saved in the Beacon.
    bool change_beacon_name(BeaconID id, std::string const& newname);

    // Estimate of performance: O(1)
    // Short rationale for estimate: The iterator for this id in _sortedColors
    // is saved in the Beacon.
    bool change_beacon_color(BeaconID id, Color newcolor);

    // Estimate of performance: O(log n) where n is the size of
    // target Beacon's sources set (small).
    // Short rationale for estimate: std::set::insert cppreference.com
    bool add_lightbeam(BeaconID sourceid, BeaconID targetid);

    // Estimate of performance: O(n) where n is the amount of light-
    // sources (small)
    // Short rationale for estimate: copying of Beacon's sources set to
    // std::vector
    std::vector<BeaconID> get_lightsources(BeaconID id);

    // Estimate of performance: O(n) where n is the amount of Beacons in
    // the outbeam
    // Short rationale for estimate: Going through Beacons' targetIDs linearly
    // starting from the id's Beacon and appending them to a vector.
    std::vector<BeaconID> path_outbeam(BeaconID id);

    /*
     * Estimate of performance: O(n + log m) where n is id Beacon's sources.size
     * and (small) m is targetID Beacon's sources.size (small).
     *
     * Short rationale for estimate: Linear assignment for all of sources
     * elements, std::set::erase(const key_type &key) cppreference.com from
     * targetID Beacon's sources-set and as std::multimap::erase(iter) and
     * std::unordered_map::erase(iter) are constant (cppreference.com).
    */
    bool remove_beacon(BeaconID id);

    /*
     * Estimate of performance: O(n + m) where n is the size of the tree id's
     * Beacon is the root of and m is the depth of the deepest leaf in that
     * tree.
     *
     * Short rationale for estimate: Go through every path from sources to
     * sources and so on and return the deepest leaf and from that leaf go
     * back up to the id's Beacon and save the path to a vector.
     *
     * Uses getDeepestLeaf for finding the deepest Leaf.
    */
    std::vector<BeaconID> path_inbeam_longest(BeaconID id);

    /*
     * Estimate of performance: O(n) ~ small log(N), where n is the amount
     * of nodes in the subtree the id's Beacon is the root of and N is the size
     * of the maps.
     *
     * Short rationale for estimate: Going through the entire tree the id's
     * Beacon is the root of and calculating totalColor for all its children
     * and finally for the Beacon itself.
     *
     * Explanation for small log(N): Excel LINEST approximation with N values
     * from 10 to 4 million gave an evalution of approximately 3/1000*log(N).
     * The plot of the cmd-times as a function of N also looks exactly like a
     * logarithmic curve.
     *
     * Uses totalColorRecursion for the tree traversing and calculation.
    */
    Color total_color(BeaconID id);

    // Phase 2 operations

    // Estimate of performance: O(n log n)
    // Short rationale for estimate: Copies contents of _xPoints to a vector
    // and sorts it.
    std::vector<Coord> all_xpoints();

    // Estimate of performance: O(log n + log m + log o) where n is size of
    // _allFibres, m is size of start XPoint's fibres set and o is the size of
    // end XPoint's fibres set. m and o are very small compared to n.

    // Short rationale for estimate: std::map::insert cppreference.com and
    // std::set::insert cppreference.com

    // Uses addFibre for the actual adding after checks.
    bool add_fibre(Coord xpoint1, Coord xpoint2, Cost cost);

    // Estimate of performance: O(m) where m is the size of Coord's
    // Xpoint's fibres set. Correct order is guaranteed by the set.
    // Short rationale for estimate: Copying from the set to a vector.
    std::vector<std::pair<Coord, Cost>> get_fibres_from(Coord xpoint);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Copy fibres from _allFibres set. The correct
    // order is guaranteed by the FibreCompare comparison operator.
    std::vector<std::pair<Coord, Coord>> all_fibres();

    // Estimate of performance: O(log n + log m + log o) ** same as add_fibre **
    // Short rationale for estimate: std::set::find cppreference.
    bool remove_fibre(Coord xpoint1, Coord xpoint2);

    // Estimate of performance: O(n) where n is the size of _xPoints and
    // _allFibres
    // Short rationale for estimate: std::set::clear and
    // std::unordered_map::clear
    void clear_fibres();

    /*
     * Estimate of performance: O(V + E) or O( (V + E) log V ) where V is the
     * amount of XPoints and E is the amount of Fibres.
     * V == _xPoints.size()
     * E = _allFibres.size()
     *
     * Short rationale for estimate: Uses BFS or Dijkstra, whichever is valid
     * at the time, to route a path. If neither are valid BFS-search is called
     * as it's faster.
     *
     * As BFS or Dijkstra are used this algorithm's time complexity shrinks
     * to O(V) if add_fibre or remove_fibre is not successfully completed during
     * consecutive calls. V is the amount of nodes in least-xpoints-path or
     * fastest-path. This behaviour is explained below (route_least_xpoints).
     *
     * Uses in order: rFastestOrShortest, initRouteSearch, ->*
     *    * doBFSAndReset, getAdjacent, resetAllColors and routeFromTo -- OR --
     *    * doDijkstraAndReset, getAdjacent, DijkstraRelax, resetAllColors and
     *      routeFromTo
     *
     * resetBFSorDij is used if needed (when neither search is valid).
     */
    std::vector<std::pair<Coord, Cost>> route_any(Coord fromxpoint,
                                                  Coord toxpoint);

    /*
     * Estimate of performance: O(V + E) ** V, E same as route_any **
     *
     * Short rationale for estimate: BFS-search on a directed graph in which
     * every node has an adjacency list.
     *
     * -- Why is all 3 routing algorithm's perftest so fast --
     * If not a single add_fibre or remove_fibre operation is successfully
     * performed during consecutive calls of this function the complexity
     * shrinks to O(V). V is the amount nodes in the shortest-path fromxpoint to
     * toxpoint. This is because the DFS-search result is stored inside XPoints'
     * and the result will stay valid if no fibres are removed or added.
     *
     * This search result reuse is the reason all of these 3 routing algorithms
     * perform so well in perftests. Perftest does not perform add- or remove-
     * operations between consecutive calls.
     * ---
     *
     * _bfsValid flag tells if the search result is valid.
     *
     * Uses in order: rFastestOrShortest, initRouteSearch, doBFSAndReset,
     * getAdjacent, resetAllColors and routeFromTo
     *
     * resetBFSorDij if needed.
     */
    std::vector<std::pair<Coord, Cost>> route_least_xpoints(Coord fromxpoint,
                                                            Coord toxpoint);

    /*
     * Estimate of performance: O( (V + E)log V ) ** V, E same as route_any **
     *
     * Short rationale for estimate: Djikstra's algorithm on a graph in which
     * every node has an adjacency list. STL priority_queue is used which
     * provides a binary heap implementation.
     *
     * This method's time complexity also shrinks to O(V) with same conditions
     * as route_least_xpoints. V is the amount of nodes in the fastest-path
     * fromx to tox.
     *
     * _dijkstraValid flag tells the validity of the result.
     *
     * Uses in order: rFastestOrShortest, initRouteSearch, doDjikstraAndReset,
     * getAdjacent, DijkstraRelax, resetAllColors and routeFromTo
     *
     * resetBFSorDij if needed.
     */
    std::vector<std::pair<Coord, Cost>> route_fastest(Coord fromxpoint,
                                                      Coord toxpoint);

    // Estimate of performance: O(V + E) ** V, E same as route_any **
    //
    // Short rationale for estimate: Uses DFS-search to find the cycle. As
    // adjacency lists are used the complexity for DFS is O(V + E)

    // Uses in order: doDFSAndReset, DFSPushAdjacent, getAdjacent, copyCycle
    // and resetDFS
    std::vector<Coord> route_fibre_cycle(Coord startxpoint);

    /*
     * Estimate of performance:
     * O( (V + E) log V + log S + 2 (V + S - 1) + (R * O(remove_fibre)) + V)
     *
     * == O ( (V + E) log V + S + R * O(remove_fibre) )
     *
     * Where V, E same as route_any, S is amount of nodes in spanning forest,
     * O(remove_fibre) is the time complexity of remove_fibre and R is the
     * nodes that are NOT in the spanning tree.
     *
     * As said the spanning forest can consist of many graphs as _xPoints is
     * iterated with the Prim's until every single XPoint is black.
     *
     * Short rationale for estimate:
     * Perform Prims's algorithm on a graph with adjacency list --
     *      O( (V + E) log V).
     * During the algorithm insert to spanningFibres set --
     *      log S.
     * After Prim's find the difference of _allFibres and spanningFibres using
     * std::set_difference --
     *      O( 2 (V + S - 1) ) cppreference.com.
     * Call remove_fibre for each of the fibres that are not in spanning tree
     * (result of the difference).
     *      O(remove_fibre) * R
     * At the very end reset XPoints colors and usedFibres
     *      V
     *
     * Uses in order: MSTTreePrims, trimPushAdj, trimTree, resetColorsAndUFibres
     */
    Cost trim_fibre_network();

private:
    // Store information when each of the searches are valid. Invalidated when
    // addFibre or removeFibre are successful. Variables are correspondingly
    // validated when doDijkstraAndReset or doBFSAndReset is called.
    bool _dijkstraValid;
    bool _bfsValid;

    // Datastructures

    std::unordered_map<BeaconID, Beacon> _beacons;
    std::multimap<std::string, BeaconID> _sortedNames;
    std::multimap<int, BeaconID> _sortedBrightness;

    // Set for Fibres supporting heterogenenous find with Coord.
    std::set<Fibre, FibreCompare> _allFibres;

    // Datastructure for fibre intersections
    std::unordered_map<Coord, XPoint, CoordHash> _xPoints;


    // Aliases for longer types that are used

    // Iterator type for _beacons, used in totalColorRecursion and getDeepestLeaf
    using beaconIter = std::unordered_map<BeaconID, Beacon>::iterator;

    // Type for _fibres and _xPoints iterator
    using fibreIter = std::set<Fibre, FibreCompare>::iterator;

    // pair type for getDeepestLeaf and for coordinate pair
    using pIntID = std::pair<int, BeaconID>;
    using CoordRefPair = std::pair<Coord&, Coord&>;

    // Type for contents of deque in DFS-search
    using XptrCost = std::pair<XPoint*, const Cost>;

    // Type of pair inside trimTree's priority queue
    using pQuePair = std::pair<Cost, XPoint*>;

    // Type for priority queue used in Dijkstra/Trim
    using dijPQueue = std::priority_queue<XPoint*, std::vector<XPoint*>, PQDijComp>;
    using trimPQue = std::priority_queue<pQuePair, std::vector<pQuePair>, PQTrimComp>;


    // Private methods - break down public ones into submethods/algorithms.

    // Combines getMin- and getMaxBrighness
    BeaconID minOrMaxBrightness(const bool max);

    // Tree traversal algorithms
    Color totalColorRecursion(const beaconIter bIter);
    pIntID getDeepestLeaf(const beaconIter bIter, pIntID maxPair = {0, NO_ID});


    // Helper methods for phase 2's public ones

    // After add_fibre checks conditions, this method does the actual adding.
    bool addFibre(const Coord &from, const Coord &to, const int cost);

    // Fibre is always stored with the smaller coordinates as start.
    std::pair<bool, fibreIter> fibresFind(CoordRefPair &cPair);

    // Initialises route_any, route_least_xpoints and route_fastest
    std::pair<XPoint*, XPoint*> initRouteSearch(const Coord &from,
                                                const Coord &to);
    // Combines route_least_xpoints and route_fastest
    std::vector<std::pair<Coord, Cost>> routeFastOrShort(const Coord& from,
                                                           const Coord& to,
                                                           const bool bfs);

    // -- Search algorithms --

    // Performs a DFS-search to the XPoint graph.
    std::vector<Coord> doDFSAndReset(XPoint* start);

    // Executes BFS/Dijkstra on the XPoint graph
    void doBFS(XPoint *xPtr);
    void doDijkstra(XPoint* start);
    void dijkstraRelax(XPoint* check, const XPoint* min, const Cost fCost);

    // Finds the MST Tree. Is called until every XPoint is black.
    void MSTPrims(XPoint* start,
                      std::set<Fibre, FibreCompare> &spanningFibres,
                      Cost &allGraphsCost);


    // -- Helper methods for the algorithms --

    // Gets adjacent XPoint based on the direction of the Fibre.
    XPoint* getAdjacent(const Fibre* fibre, const XPoint* from);

    // Pushes the adjacent XPoints to deq and resetColorsOf.
    bool DFSPushAdjacent(std::deque<XptrCost> &deq,
                         std::vector<XPoint*> &resetColorsOf);

    // Resets colors and hereFrom-ptrs after DFS-search.
    void resetDFS(const std::vector<XPoint*> &xPtrs);

    // Resets d and pi pointers for BFS or Dijkstra.
    void resetBFSorDij(const bool bfs);

    // Reset colors of all XPoints after BFS/Dijkstra. Also resets usedFibres
    // as trimTreePrims uses those ptrs.
    void resetColorsAndUFibres();

    // Logic for adjacent pushing in Dijkstras
    void dijkstraPushAdj(dijPQueue &pQueue, const XPoint *minNode);

    // Pushes adjacent to priority queue for MSTTreePrims
    void trimPushAdj(trimPQue &pQue, const XPoint* minNode);

    // Performs the trimming operation
    void trimTree(const std::set<Fibre, FibreCompare> &inSpanning);



    // -- Routing algorithms to-be-used after searching --

    // Recursive method to route the shortest/fastest from 2 coordinates after
    // BFS/Dijkstra
    bool routeFromTo(const XPoint* fromPtr,
                     const XPoint* toPtr,
                     std::vector<std::pair<Coord, Cost>> &route,
                     const bool bfs);

    // Copies the cycle path for route_fibre_cycle after DFS has found one.
    // cycleSecond is needed to indicate the second node in the cycle. Without
    // it copyCycle would loop infinitely as all the hereFrom pointers would
    // create a loop.
    void copyCycle(std::deque<XptrCost> &deq,
                   std::vector<Coord> &toVec) const;

};

#endif // DATASTRUCTURES_HH
