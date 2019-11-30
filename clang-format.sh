# -*- yaml -*-

# This file determines clang-format's style settings; for details, refer to
# http://clang.llvm.org/docs/ClangFormatStyleOptions.html

BasedOnStyle:  Google

Language: Cpp

# Force pointers to the type for C++.
DerivePointerAlignment: false
PointerAlignment: Left

# Specify the #include statement order.  This implements the order mandated by
# the Google C++ Style Guide: related header, C headers, C++ headers, library
# headers, and finally the project headers.
#
# To obtain updated lists of system headers used in the below expressions, see:
# http://stackoverflow.com/questions/2027991/list-of-standard-header-files-in-c-and-c/2029106#2029106.
# IncludeCategories:
#   # Spacers used by drake/tools/formatter.py.
#   - Regex:    '^<clang-format-priority-15>$'
#     Priority: 15
#   - Regex:    '^<clang-format-priority-25>$'
#     Priority: 25
#   - Regex:    '^<clang-format-priority-35>$'
#     Priority: 35
#   - Regex:    '^<clang-format-priority-45>$'
#     Priority: 45
#   # C system headers.
#   - Regex:    '^[<"](aio|arpa/inet|assert|complex|cpio|ctype|curses|dirent|dlfcn|errno|fcntl|fenv|float|fmtmsg|fnmatch|ftw|glob|grp|iconv|inttypes|iso646|langinfo|libgen|limits|locale|math|monetary|mqueue|ndbm|netdb|net/if|netinet/in|netinet/tcp|nl_types|poll|pthread|pwd|regex|sched|search|semaphore|setjmp|signal|spawn|stdalign|stdarg|stdatomic|stdbool|stddef|stdint|stdio|stdlib|stdnoreturn|string|strings|stropts|sys/ipc|syslog|sys/mman|sys/msg|sys/resource|sys/select|sys/sem|sys/shm|sys/socket|sys/stat|sys/statvfs|sys/time|sys/times|sys/types|sys/uio|sys/un|sys/utsname|sys/wait|tar|term|termios|tgmath|threads|time|trace|uchar|ulimit|uncntrl|unistd|utime|utmpx|wchar|wctype|wordexp)\.h[">]$'
#     Priority: 20
#   # C++ system headers (as of C++17).
#   - Regex:    '^[<"](algorithm|any|array|atomic|bitset|cassert|ccomplex|cctype|cerrno|cfenv|cfloat|charconv|chrono|cinttypes|ciso646|climits|clocale|cmath|codecvt|complex|condition_variable|csetjmp|csignal|cstdalign|cstdarg|cstdbool|cstddef|cstdint|cstdio|cstdlib|cstring|ctgmath|ctime|cuchar|cwchar|cwctype|deque|exception|execution|filesystem|forward_list|fstream|functional|future|initializer_list|iomanip|ios|iosfwd|iostream|istream|iterator|limits|list|locale|map|memory|memory_resource|mutex|new|numeric|optional|ostream|queue|random|ratio|regex|scoped_allocator|set|shared_mutex|sstream|stack|stdexcept|streambuf|string|string_view|strstream|system_error|thread|tuple|type_traits|typeindex|typeinfo|unordered_map|unordered_set|utility|valarray|variant|vector)[">]$'
#     Priority: 30
#   # Other libraries' h files (with angles).
#   - Regex:    '^<'
#     Priority: 40
#   # Your project's h files.
#   - Regex:    '^"drake'
#     Priority: 50
#   # Other libraries' h files (with quotes).
#   - Regex:    '^"'
#     Priority: 41

