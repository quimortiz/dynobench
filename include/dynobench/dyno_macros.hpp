#pragma once

#include <boost/stacktrace/stacktrace.hpp>
#include <boost/stacktrace/stacktrace_fwd.hpp>
#include <iostream>
#include <sstream>

#define NAMEOF(variable) #variable

#define VAR_WITH_NAME(variable) variable, #variable

#define NAME_AND_STRING(variable)                                              \
  std::make_pair(#variable, std::to_string(variable))

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define AT __FILE__ ":" TOSTRING(__LINE__)

#define STR(x, sep) #x << sep << x

#define STR_(x) #x << ": " << x

#define CSTR_(x) std::cout << #x << ": " << x << std::endl;

#define STRY(x, out, be, af) out << be << #x << af << x << std::endl

#define FMT_E                                                                  \
  Eigen::IOFormat(6, Eigen::DontAlignCols, ",", ",", "", "", "[", "]")

#define STR_VV(x, af) #x << af << x.format(FMT_E)

#define STR_V(x) #x << ": " << x.format(FMT_E)

#define CSTR_V(x) std::cout << STR_V(x) << std::endl;

std::string inline add_stacktrace(const std::string &msg) {

  return "\nMSG: " + msg + "\n" + "STACKTRACE: \n" +
         boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n";
}

#define CHECK(A, msg)                                                          \
  if (!A) {                                                                    \
    std::cout << "CHECK failed: '" << #A << " " << A << " '"                   \
              << " -- " << add_stacktrace(msg) << "AT: " << AT << std::endl;   \
    throw std::runtime_error(add_stacktrace(msg));                             \
  }

#define WARN(A, msg)                                                           \
  if (!A) {                                                                    \
    std::cout << "CHECK failed: '" << #A << " " << A << " '"                   \
              << " -- " << msg << std::endl;                                   \
  }

#define DYNO_CHECK_EQ(A, B, msg)                                               \
  if (!(A == B)) {                                                             \
    std::cout << "DYNO_CHECK_EQ failed: '" << #A << "'=" << A << " '" << #B    \
              << "'=" << B << " -- " << add_stacktrace(msg) << std::endl       \
              << "AT: " << AT << std::endl;                                    \
    throw std::runtime_error(add_stacktrace(msg));                             \
  }

#define CHECK_NEQ(A, B, msg)                                                   \
  if (A == B) {                                                                \
    std::cout << "CHECK_NEQ failed: '" << #A << "'=" << A << " '" << #B        \
              << "'=" << B << " -- " << add_stacktrace(msg) << std::endl       \
              << "AT: " << AT << std::endl;                                    \
    throw std::runtime_error(add_stacktrace(msg));                             \
  }

#define DYNO_DYNO_CHECK_GEQ(A, B, msg)                                         \
  if (!(A >= B)) {                                                             \
    std::cout << "DYNO_DYNO_CHECK_GEQ failed: '" << #A << "'=" << A << " '"    \
              << #B << "'=" << B << " -- " << add_stacktrace(msg) << std::endl \
              << "AT: " << AT << std::endl;                                    \
    throw std::runtime_error(add_stacktrace(msg));                             \
  }

#define DYNO_WARN_GEQ(A, B, msg)                                               \
  if (!(A >= B)) {                                                             \
    std::cout << "DYNO_WARN_GEQ failed: '" << #A << "'=" << A << " '" << #B    \
              << "'=" << B << " -- " << msg << std::endl                       \
              << AT << std::endl;                                              \
    std::cerr << "DYNO_WARN_GEQ failed: '" << #A << "'=" << A << " '" << #B    \
              << "'=" << B << " -- " << msg << std::endl                       \
              << AT << std::endl;                                              \
  }

#define DYNO_CHECK_LEQ(A, B, msg)                                              \
  if (!(A <= B)) {                                                             \
    std::cout << "DYNO_CHECK_LEQ failed: '" << #A << "'=" << A << " '" << #B   \
              << "'=" << B << " -- " << add_stacktrace(msg) << std::endl       \
              << "AT: " << AT << std::endl;                                    \
    throw std::runtime_error(add_stacktrace(msg));                             \
  }

#define DYNO_CHECK_GE(A, B, msg)                                               \
  if (!(A > B)) {                                                              \
    std::cout << "DYNO_CHECK_GE failed: '" << #A << "'=" << A << " '" << #B    \
              << "'=" << B << " -- " << add_stacktrace(msg) << "AT: " << AT    \
              << std::endl;                                                    \
    throw std::runtime_error(add_stacktrace(msg));                             \
  }

#define DYNO_CHECK_SEQ(A, B, msg)                                              \
  if (!(A <= B)) {                                                             \
    std::cout << "DYNO_CHECK_SEQ failed: '" << #A << "'=" << A << " '" << #B   \
              << "'=" << B << " -- " << add_stacktrace(msg) << "AT: " << AT    \
              << std::endl;                                                    \
    throw std::runtime_error(add_stacktrace(msg));                             \
  }

#define ERROR_WITH_INFO(msg)                                                   \
  throw std::runtime_error(std::string("--ERROR-- ") + __FILE__ +              \
                           std::string(":") + std::to_string(__LINE__) +       \
                           " \"" + add_stacktrace(msg) + "\"\n");

#define NOT_IMPLEMENTED ERROR_WITH_INFO("not implemented")

#define WARN_WITH_INFO(msg)                                                    \
  std::cout << __FILE__ + std::string(":") + std::to_string(__LINE__) + "\"" + \
                   msg + "\"\n"                                                \
            << std::endl;
