#!/bin/bash

find envs/ -iname "*.yaml" | xargs npx prettier --write
find models/ -iname "*.yaml" | xargs npx prettier --write


find src/ -iname "*.h" -o  -iname "*.cpp" -o  -iname "*.hpp" | xargs clang-format-15  -i
find include/ -iname "*.h" -o  -iname "*.cpp" -o  -iname "*.hpp" | xargs clang-format-15  -i
find test/ -iname "*.h" -o  -iname "*.cpp" -o  -iname "*.hpp" | xargs clang-format-15  -i

find utils/viewer -name '*.py' -exec autopep8 --in-place --ignore E402 --aggressive '{}' \;




