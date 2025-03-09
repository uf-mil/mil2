all_tests_sources = [
  'binary_compatibility_test.cpp',
  'calcsize_test.cpp',
  'format_test.cpp',
  'pack_test.cpp',
  'string_test.cpp',
  'unpack_test.cpp',
]

foreach source: all_tests_sources
  target_name = source.replace('.cpp', '')
  exe = executable(target_name, [source, 'main.cpp'],
    include_directories: includes,
    cpp_args: compile_args + ['-DWITH_CONSTEXPR_REQUIRE'],
    dependencies: dependencies)

  test_name = target_name.replace('_test', '')
  test(test_name, exe)
endforeach

sources_with_main = [
  'demo.cpp',
  'string_literal_test.cpp',
  'newpack_test.cpp',
]

foreach source: sources_with_main
  target_name = source.replace('.cpp', '')
  exe = executable(target_name, source,
    include_directories: includes,
    cpp_args: compile_args + ['-DWITH_CONSTEXPR_REQUIRE'],
    dependencies: dependencies)

  test_name = target_name.replace('_test', '')
  test(test_name, exe)
endforeach
