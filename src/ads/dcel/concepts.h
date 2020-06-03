#pragma once

#include "ads/ccpp/typedefs.h"

#include <boost/type_traits.hpp>

#include <type_traits>

namespace ads
{
namespace dcel
{

/*! Concept checker for functors passed to the Dcel::forAllSegments methods
 *
 * Must have the following traits
 * * Move-assignable
 * * Invokable with ConstReferringSegment
 */
template <class Functor> struct ForEachFunctorConcept
{
    static_assert(std::is_move_assignable<Functor>::value, "Dcel Functor is not move-assignable");

    Functor f1;
    Functor f2;

    // Points to use in check
    Point2d p1, p2;

    BOOST_CONCEPT_USAGE(ForEachFunctorConcept)
    {
        // Fails if not move-assignable
        f1 = std::move(f2);

        // Fails if not invokable correctly
        const boost::geometry::model::referring_segment<const Point2d> segment(p1, p2);
        f1(segment);
    }
};

/*! Concept checker for strategies passed to Dcel::transform
 *
 * Must have the following traits
 *  * Have an apply method matching <bool (const ccpp::geometry::Point2d&, ccpp::geometry::Point2d&) const>
 */
template <class Strategy> struct TransformStrategyConcept
{
    const Strategy s;

    // Points to use in check
    const ccpp::geometry::Point2d p1;
    ccpp::geometry::Point2d p2;

    BOOST_CONCEPT_USAGE(TransformStrategyConcept)
    {
        bool result = s.apply(p1, p2);
        boost::ignore_unused(result);
    }
};
}
}
