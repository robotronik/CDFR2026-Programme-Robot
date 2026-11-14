# Code style

Write code a first-year member can read. When in doubt, pick the simpler version.
These rules apply to new and edited code; don't rewrite old code just for style.

## C first, C++ where it helps

- Default: free functions and plain structs for data.
- Use a class only when something owns state with a lifetime (hardware link,
  camera, thread, database) or needs polymorphism (actions, strategies).
- Inheritance only through the existing interfaces (`VirtualAction`, `VirtualStrategy`).
- Use the standard library: `std::string`, `std::vector`, `std::unique_ptr`.
  No raw `new`/`delete`, no custom templates.
- No exceptions in robot code: return `bool` and log the error. Catch what
  libraries throw at the call site.
- `auto` only when the type is obvious (iterators, `make_unique`). Lambdas only
  for callbacks (REST routes, threads).

## Naming

- Classes and types: PascalCase (`DriveControl`). C-style structs/enums: `name_t` (`position_t`).
- Functions and variables: camelCase (`moveServoAndWait`).
- Constants and macros: UPPER_SNAKE (`SERVO_NUM_6`). Use `constexpr` for new constants.
- Files: named after their class (`Lidar.cpp`) or module (`navigation.cpp`). New headers: `.hpp`.

## Structure

- `#pragma once`; include as `"module/File.hpp"`.
- Pass dependencies through constructors or function arguments, not new globals.
- Short functions, early returns, at most ~3 levels of nesting.

## Output and comments

- Log with `LOG_DEBUG` / `LOG_INFO` / `LOG_WARNING` / `LOG_ERROR`, never `printf` or `std::cout`.
- English names and comments. Comment *why*, not what.

## Formatting

[`.clang-format`](../.clang-format) matches the existing code (4 spaces, braces
on the same line). Format the files you touch: `clang-format -i <file>`, or use
your editor's "Format Document" (clangd reads this file).
