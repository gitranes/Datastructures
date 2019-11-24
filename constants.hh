#ifndef CONSTANTS_HH
#define CONSTANTS_HH

#include <utility>
#include <limits>
#include <map>
#include <string>

/**
  * Constants and types. NO_COLOR and NO_COORD were moved to structs.hh
  * because they required the corresponding type definition.
  */

// Type for beacon IDs
using BeaconID = std::string;

// Return value for cases where required beacon was not found
BeaconID const NO_ID = "----------";

// Iterator types for sorted multimaps
using sNameIter = std::multimap<std::string, BeaconID>::iterator;
using sColorIter = std::multimap<int, BeaconID>::iterator;

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
std::string const NO_NAME = "-- unknown --";

// Type for light transmission cost (used only in the second assignment)
using Cost = int;

// Return value for cases where cost is unknown
Cost const NO_COST = NO_VALUE;

#endif // CONSTANTS_HH
