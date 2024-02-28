#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

class Printer {
public:
  enum PrintLevel { ALL = 0, DEBUG = 1, INFO = 2, WARNING = 3, ERROR = 4};
  static void setPrintLevel(const std::string &level);
  static void setPrintLevel(PrintLevel level);
  static void debugPrint(PrintLevel level, const char location[], const char line[], const char *format, ...);
  static PrintLevel current_print_level;

private:
  static constexpr uint32_t MAX_FILE_PATH_LEGTH = 30;
};

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define PRINT_ALL(x...) Printer::debugPrint(Printer::PrintLevel::ALL, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_INFO(x...) Printer::debugPrint(Printer::PrintLevel::INFO, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_WARNING(x...) Printer::debugPrint(Printer::PrintLevel::WARNING, __FILE__, TOSTRING(__LINE__), x);
#define PRINT_ERROR(x...) Printer::debugPrint(Printer::PrintLevel::ERROR, __FILE__, TOSTRING(__LINE__), x);

#ifdef XSLAM_DEBUG
#define PRINT_DEBUG(x...) Printer::debugPrint(Printer::PrintLevel::DEBUG, __FILE__, TOSTRING(__LINE__), x);
#define runtime_assert(condition, message)                             \
  do {                                                                 \
    if (!(condition)) {                                                \
      log_error("Assertion failed at " __FILE__                        \
              ":%d : %s\nWhen testing condition:\n    %s",             \
              __LINE__, message, #condition);                          \
      abort();                                                         \
    }                                                                  \
  } while (0)
#else
#define PRINT_DEBUG(...)
#define runtime_assert(...)
#endif

// 显示颜色
#define RESET "\033[0m"
#define BLACK "\033[30m"                /* Black */
#define RED "\033[31m"                  /* Red */
#define GREEN "\033[32m"                /* Green */
#define YELLOW "\033[33m"               /* Yellow */
#define BLUE "\033[34m"                 /* Blue */
#define MAGENTA "\033[35m"              /* Magenta */
#define CYAN "\033[36m"                 /* Cyan */
#define WHITE "\033[37m"                /* White */
#define REDPURPLE "\033[95m"            /* Red Purple */
#define BOLDBLACK "\033[1m\033[30m"     /* Bold Black */
#define BOLDRED "\033[1m\033[31m"       /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"     /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"    /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"   /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"     /* Bold White */
#define BOLDREDPURPLE "\033[1m\033[95m" /* Bold Red Purple */
