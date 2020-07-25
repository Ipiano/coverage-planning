#pragma once

#include <stdexcept>

namespace ads
{

//! Runtime error exception to be thrown when an assertion would normally fail.
//! Use the ASSERT macro defined here instead of a regular 'assert' to throw
//! this exception on failure. Using this assertion makes it easier to debug
//! failures of the algorithm on edge cases, and also possible to catch the
//! exception and move on in code that should not crash.
struct AssertionFailure : public std::runtime_error
{
    AssertionFailure(const std::string& str) : std::runtime_error(str) {}
};

#define ASSERT_Q(x) #x
#define ASSERT_QUOTE(x) ASSERT_Q(x)
#define ASSERT(x)                                                                                      \
    if (!(x))                                                                                          \
    {                                                                                                  \
        throw AssertionFailure(ASSERT_QUOTE(__FILE__) ":" ASSERT_QUOTE(__LINE__) "-" ASSERT_QUOTE(x)); \
    }
}
