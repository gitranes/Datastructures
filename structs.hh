#ifndef STRUCTS_HH
#define STRUCTS_HH

#include "constants.hh"

#include <climits>
#include <set>
#include <functional>

// Different types of structs used in mainly in Datastructures class


// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

// Example: Defining < for Coord so that it can be used as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}

struct CoordHash
{
    std::size_t operator()(Coord xy) const
    {
        auto hasher = std::hash<int>();
        auto xhash = hasher(xy.x);
        auto yhash = hasher(xy.y);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Type for color (RGB)
struct Color
{
    // Constructor for Color
    Color(const int r, const int g, const int b) :
        r(r), g(g), b(b) {}
    int r = NO_VALUE;
    int g = NO_VALUE;
    int b = NO_VALUE;

    int brightness() const
    {
        return 3*r + 6*g + b;
    }
};

// Return value for cases where color was not found
Color const NO_COLOR = {NO_VALUE, NO_VALUE, NO_VALUE};

// Equality and non-equality comparisons for Colors
inline bool operator==(Color c1, Color c2) { return c1.r == c2.r && c1.g == c2.g && c1.b == c2.b; }
inline bool operator!=(Color c1, Color c2) { return !(c1==c2); }

// Addition assignment operator for Color. Used in totalColorRecursion.
inline void operator+=(Color &c1, const Color &c2)
{
    c1.r += c2.r;
    c1.g += c2.g;
    c1.b += c2.b;
}

// Division operator for Color. Used in totalColorRecursion.
inline Color operator/ (const Color &c1, const int denom)
{
    return Color(c1.r / denom, c1.g / denom, c1.b / denom);
}

// Type for beacons
struct Beacon
{
    // Constructor for Beacon. Assigns given parameters to the attributes.
    Beacon(const std::string &name, const Coord &xy, const Color &color,
           const sNameIter namesIter, const sColorIter colorsIter)
        : name(name), xy(xy), color(color),
          namesIter(namesIter),
          colorsIter(colorsIter) {}

    // Id of the beacon this is pointing to
    BeaconID targetID = NO_ID;
    // Ids of the beacons pointing to this
    std::set<BeaconID> sources;

    // Beacon data
    std::string name;
    Coord xy;
    Color color;

    // Iterators to _sortedNames and _sortedColors for this ID
    sNameIter namesIter;
    sColorIter colorsIter;
};

// Forward declaration
struct XPoint;

// Type for light fibres
struct Fibre
{
    // Fibre cost constructor
    Fibre(const Cost cost) : cost(cost) {}

    // Pointers to Xpoints inside _xPoints
    XPoint* start;
    XPoint* end;

    Cost cost;
};

// Allowing heterogeneous find (Coord or Fibre). Used by _allFibres set.
struct FibreCompare
{
    using is_transparent = std::true_type;
    using constCoordPair = const std::pair<const Coord&, const Coord&>;

    bool operator()(const Fibre &lhs, const Fibre &rhs) const;

    // Transparent comparison with Coord start-end pair and Fibre.
    bool operator()(const Fibre &lhs, constCoordPair& cPair) const;
    bool operator()(constCoordPair& cPair, const Fibre &rhs) const;
};

// Allowing heterogeneous find (Coord or Fibre*). Used by fibres set in XPoint.
struct FibrePtrCompare
{
    using is_transparent = std::true_type;
    using constCoordPair = FibreCompare::constCoordPair;

    bool operator()(const Fibre* const lhs, const Fibre* const rhs) const;

    // Transparent comparison with Coord start-end pair and Fibre*.
    bool operator()(const Fibre* const lhs, constCoordPair& cPair) const;
    bool operator()(constCoordPair& cPair, const Fibre* const rhs) const;
};

// Type for XPoint colors. Used in BFS/DFS/Djikstra.
enum XColor
{
    White, Gray, Black
};

// Type for Fibre intersections
struct XPoint
{
    // Constructor for a new Xpoint
    XPoint(const Coord &coord) :
        coord(coord), piBfs(nullptr), dBfs(INT_MAX),
        piDij(nullptr), dDij(INT_MAX), hereFrom(nullptr),
        color(White), usedFibre(nullptr)
    {}

    // XPoint is too expensive to copy so delete the implicit copy constructor.
    XPoint(const XPoint &copy) = delete;

    // Forbid modifications as _allFibres set is ordered based on this.
    const Coord coord;

    // Stores BFS-search results. pi is the shortest path to a certain XPoint
    // and d is the cost of it.
    const XPoint* piBfs;
    Cost dBfs;

    // Stores Dijkstra-algorithm results.
    const XPoint* piDij;
    Cost dDij;

    // Used in find_cycle to print the path.
    const XPoint* hereFrom;

    // See XColor (above)
    XColor color;

    // Datastructure for pointers to fibres inside _allFibres.
    std::set<const Fibre*, FibrePtrCompare> fibres;

    // Stores the fibre we used to path to this. Used in trimTreepPrims.
    const Fibre* usedFibre;
};

// Helper functions for tying Fibre's coordinates to pairs. std::tie returns
// a tuple of non-const lvalue refs so it can't be used here. Note C++17 class
// template argument deduction is used here.
inline const std::pair<const Coord&, const Coord&>
    tieFCoords(const Fibre& fibre)
{
    return std::pair(std::cref(fibre.start->coord),
                     std::cref(fibre.end->coord));
}

inline const std::pair<const Coord&, const Coord&>
    tieFPtrCoords(const Fibre* const fibrePtr)
{
    return std::pair(std::cref(fibrePtr->start->coord),
                     std::cref(fibrePtr->end->coord));
}

struct PQDijComp
{
    // Operator for min-priority queue. Used in Dijkstra's-algorithm.
    inline bool operator()(const XPoint* const lhs, const XPoint* const rhs)
    {
        return lhs->dDij > rhs->dDij;
    }
};

struct PQTrimComp
{
    using pQuePair = std::pair<const Cost, const XPoint*>;

    // Operator for min-priority queue for Prim's algorithm. Used in trimTree.
    // Pairs are sorted based on their fibre cost.
    inline bool operator()(const pQuePair &lhs, const pQuePair &rhs)
    {
        if (lhs.first == rhs.first)
        {
            // If costs are equal return false
            return false;
        }
        else {
            return lhs.first > rhs.first;
        }
    }
};


#endif // STRUCTS_HH
