# build-all by default
default: build-all

# show this information
help:
    @just --list

# setup build env
setup:
    meson setup --reconfigure build

# build all
build-all:
    meson compile -C build

# build a specific target
build TARGET:
    meson compile -C build {{TARGET}}

# test all
test-all:
    meson test -C build

# run a specific test
test TEST:
    meson test -C build {{TEST}} --verbose

# run pre-commit
pre-commit:
    pre-commit run -a

# install(dry-run)
dry-run:
    meson install -C build --no-rebuild --only-changed --dry-run

# install library
install:
    meson install -C build --no-rebuild --only-changed

build-cmake:
    cmake --build build

build-cmake-setup:
    cmake -GNinja -Bbuild -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

format:
    fd -e hpp -e cpp . -E ./build -E ./subprojects/ -x clang-format -i {}

replace XXX YYY:
    fd -e hpp -e cpp . -E ./build -E ./subprojects/ -x sed -i 's/'$(echo {{XXX}} | sed 's/\//\\\//g')'/'$(echo {{YYY}} | sed 's/\//\\\//g')'/g' {}
