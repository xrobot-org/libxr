# XRobot 生成头文件的内容检查（cmake/XRobotFreshness.cmake）。
# The content check of the XRobot generated header (cmake/XRobotFreshness.cmake).
#
# 用法 / Usage:
#   cmake -DWORK=<dir> -DCHECK=<XRobotFreshness.cmake> -P xrobot_freshness.cmake
#
# 摘要与 xrobot 的 test_the_digest_matches_the_libxr_check 用同一组文件，期望同一个值；两边
# 算法不同时，构建和 xrobot describe 对同一个头文件给出不同的结论。
# The digest uses the same files as xrobot's test_the_digest_matches_the_libxr_check and
# expects the same value; differing algorithms would make the build and xrobot describe
# disagree about one header.

file(REMOVE_RECURSE "${WORK}")
set(_user "${WORK}/User")
set(_header "${_user}/xrobot_main.hpp")
set(_lock "${WORK}/xrobot.lock")
set(_entry "${_user}/app_main.cpp")
set(_module "${WORK}/Modules/team/Foo/Foo.hpp")
# 入口源文件：换行为 CRLF、跨行的调用、类型中带括号，注释中的调用也计入摘要。
# The entry: CRLF line ends, a call over two lines, parentheses in a type, and a call inside
# a comment, which the digest counts too.
string(
  CONCAT _entry_text
         "#include \"xrobot_main.hpp\"\r\n// 入口\r\nint main() {\r\n  XR_REGISTER(pin, int);\r\n"
         "  XR_REGISTER(table, /* 表 */\r\n              decltype(storage[0]));\r\n"
         "  // XR_REGISTER(old, int);\r\n  XROBOT_MAIN();\r\n}\r\n"
)
set(_module_text "class Foo { public: Foo() {} };\n")
string(
  CONCAT _inputs
         "// xrobot: config \"产品/英雄.yaml\"\n// xrobot: depends \"../xrobot.lock\"\n"
         "// xrobot: entry \"app_main.cpp\"\n// xrobot: depends \"../Modules/team/Foo/Foo.hpp\"\n"
)
set(_digest "307fb19f0ca77bf9074ff03a636161f750578e17e6ba30c99e4634fede9a30e2")

file(WRITE "${_user}/产品/英雄.yaml" "modules:\n  - module: Foo\n    id: foo\n")
file(WRITE "${_lock}" "lock: 1\n")
file(WRITE "${_entry}" "${_entry_text}")
file(WRITE "${_module}" "${_module_text}")
file(WRITE "${_header}" "#pragma once\n${_inputs}// xrobot: digest ${_digest}\n")

set(_run "Run `xrobot gen -c User/产品/英雄.yaml` in ${WORK}")

# 运行检查。expected 为空时要求通过；否则要求失败，且合并空白后的报错等于 expected。
# Run the check. An empty expected requires it to pass; otherwise it must fail with the
# message expected, compared with runs of white space collapsed.
function(expect case expected)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" "-DXROBOT_MAIN_HEADER=${_header}" "-DXROBOT_BSP_ROOT=${WORK}"
            -P "${CHECK}"
    RESULT_VARIABLE _result
    ERROR_VARIABLE _error
    OUTPUT_QUIET
  )
  string(REGEX REPLACE "^.*\\(message\\):" "" _message "${_error}")
  string(REGEX REPLACE "[ \t\r\n]+" " " _message "${_message}")
  string(STRIP "${_message}" _message)
  if(expected STREQUAL "")
    if(NOT _result EQUAL 0)
      message(FATAL_ERROR "${case}: the check failed: ${_message}")
    endif()
  elseif(_result EQUAL 0)
    message(FATAL_ERROR "${case}: the check passed, expected: ${expected}")
  elseif(NOT _message STREQUAL expected)
    message(FATAL_ERROR "${case}: expected\n  ${expected}\ngot\n  ${_message}")
  endif()
endfunction()

expect("unchanged inputs" "")

# 以前按修改时间判断，只改时间（git checkout、复制工程、时钟差）也会使构建失败。
# File times used to decide, so a new time alone (git checkout, a copy, clock skew) failed
# the build.
file(TOUCH "${_user}/产品/英雄.yaml" "${_lock}" "${_entry}" "${_module}")
expect("new file times" "")

file(APPEND "${_entry}" "// a note outside the registrations\n")
expect("entry code outside the registrations" "")

string(CONCAT _stale "[XRobot] ${_header} is stale: its inputs changed after it was "
       "generated. ${_run} before building."
)
file(APPEND "${_entry}" "XR_REGISTER(extra, int);\n")
expect("a new registration" "${_stale}")
file(WRITE "${_entry}" "${_entry_text}")

file(APPEND "${_module}" "// note\n")
expect("a changed module header" "${_stale}")
file(WRITE "${_module}" "${_module_text}")

file(REMOVE "${_lock}")
expect("a missing input"
       "[XRobot] ${_header} was generated from ${_lock}, which no longer exists. ${_run}."
)
file(WRITE "${_lock}" "lock: 1\n")

file(WRITE "${_header}" "#pragma once\n${_inputs}")
string(CONCAT _old "[XRobot] ${_header} records no digest of its inputs; it was generated "
       "by an older xrobot. ${_run}."
)
expect("a header without a digest" "${_old}")

file(REMOVE "${_header}")
expect("a missing header" "[XRobot] ${_header} is missing. Run `xrobot setup` in ${WORK}.")
