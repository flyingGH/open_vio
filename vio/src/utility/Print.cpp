#include "utility/Print.h"

Printer::PrintLevel Printer::current_print_level = PrintLevel::DEBUG;

void Printer::setPrintLevel(const std::string &level) {
  if (level == "ALL")
    setPrintLevel(PrintLevel::ALL);
  else if (level == "DEBUG")
    setPrintLevel(PrintLevel::DEBUG);
  else if (level == "INFO")
    setPrintLevel(PrintLevel::INFO);
  else if (level == "WARNING")
    setPrintLevel(PrintLevel::WARNING);
  else if (level == "ERROR")
    setPrintLevel(PrintLevel::ERROR);
  else {
    std::cout << "levels: ALL, DEBUG, INFO, WARNING, ERROR" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

void Printer::setPrintLevel(PrintLevel level) {
  Printer::current_print_level = level;
  std::cout << "Setting printing level to: ";
  switch (current_print_level) {
  case PrintLevel::ALL:
    std::cout << "ALL";
    break;
  case PrintLevel::DEBUG:
    std::cout << "DEBUG";
    break;
  case PrintLevel::INFO:
    std::cout << "INFO";
    break;
  case PrintLevel::WARNING:
    std::cout << "WARNING";
    break;
  case PrintLevel::ERROR:
    std::cout << "ERROR";
    break;
  default:
    std::cout << std::endl;
    std::cout << "levels: ALL, DEBUG, INFO, WARNING, ERROR" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::cout << std::endl;
}

void Printer::debugPrint(PrintLevel level, const char location[], const char line[], const char *format, ...) {
  if (static_cast<int>(level) < static_cast<int>(Printer::current_print_level)) {
    return;
  }

  if (static_cast<int>(Printer::current_print_level) <= static_cast<int>(Printer::PrintLevel::DEBUG)) {
    std::string path(location);
    std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
    if (base_filename.size() > MAX_FILE_PATH_LEGTH) {
      printf("%s", base_filename.substr(base_filename.size() - MAX_FILE_PATH_LEGTH, base_filename.size()).c_str());
    } else {
      printf("%s", base_filename.c_str());
    }
    printf(":%s ", line);
  }

  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
}
