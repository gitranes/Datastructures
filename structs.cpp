#include "structs.hh"

/**
 * These operators had to be defined non-inline because of circular type
 * dependencies with FibreComparison and Xpoint. std::pair's lexicographical
 * comparison is used.
 */

bool FibreCompare::operator()(const Fibre &lhs, const Fibre &rhs) const
{
    return tieFCoords(lhs) < tieFCoords(rhs);
}

bool FibreCompare::operator()(const Fibre &lhs, constCoordPair& cPair) const
{
   return tieFCoords(lhs) < cPair;
}

bool FibreCompare::operator()(constCoordPair& cPair, const Fibre &rhs) const
{
    return cPair < tieFCoords(rhs);
}

bool FibrePtrCompare::operator()(const Fibre* const lhs,
                                 const Fibre* const rhs) const
{
    return tieFPtrCoords(lhs) < tieFPtrCoords(rhs);
}

bool FibrePtrCompare::operator()(const Fibre* const lhs,
                                 constCoordPair& cPair) const
{
    return tieFPtrCoords(lhs) < cPair;
}

bool FibrePtrCompare::operator()(constCoordPair& cPair,
                                 const Fibre* const rhs) const
{
    return cPair < tieFPtrCoords(rhs);
}
