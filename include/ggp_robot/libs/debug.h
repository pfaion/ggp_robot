#ifndef __GGP_ROBOT_DEBUG_H
#define __GGP_ROBOT_DEBUG_H

std::string print_color(std::string name) {
  if(name == "black") return "\x1b[30m";
  if(name == "red") return "\x1b[31m";
  if(name == "green") return "\x1b[32m";
  if(name == "yellow") return "\x1b[33m";
  if(name == "blue") return "\x1b[34m";
  if(name == "magenta") return "\x1b[35m";
  if(name == "cyan") return "\x1b[36m";
  if(name == "white") return "\x1b[37m";
  return "\x1b[0m";
}

#define PRINT1(X) do { std::cout << X << std::endl; } while (0)
#define PRINT2(COL, X) do { std::cout << print_color(#COL) << X << "\x1b[0m" << std::endl;; } while (0)
#define GET_MACRO(_1, _2, NAME, ...) NAME
#define PRINT(...) GET_MACRO(__VA_ARGS__, PRINT2, PRINT1)(__VA_ARGS__)

#endif
