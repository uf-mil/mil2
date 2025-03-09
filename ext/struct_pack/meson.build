project('struct_pack', 'cpp',
  version: '0.0.1',
  default_options: [
    'buildtype=debug',
    'default_library=shared',
    'cpp_std=c++20',
    'warning_level=3'],
  meson_version: '>=1.2.0')

compile_args = []

includes = include_directories('include')

catch2_dep = dependency('catch2', version: ['>=2.13.8', '<3'])
dependencies = []
dependencies += catch2_dep

subdir('tests')
