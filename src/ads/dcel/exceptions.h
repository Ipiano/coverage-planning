#pragma once

#include <stdexcept>
#include <string>

namespace ads
{
namespace dcel
{
//! Base type for exceptions thrown by the Dcel class
class DcelError : public std::exception
{
    std::string m_what;

  public:
    DcelError(const std::string& what) : m_what(what) {}

    const char* what() const noexcept { return m_what.c_str(); }
};

//! Exception thrown when an invalid handle is used
class InvalidHandleError : public DcelError
{
  public:
    InvalidHandleError(const std::string& what) : DcelError(what) {}
};
}
}
