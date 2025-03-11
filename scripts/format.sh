# Get the absolute path of this script
# Source: https://stackoverflow.com/questions/59895
THIS_SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ROOT_DIR=$(realpath $THIS_SCRIPT_DIR/..)

PATHS="$ROOT_DIR/src/ $ROOT_DIR/include $ROOT_DIR/unit_tests/ $ROOT_DIR/executables/"
echo $PATHS

echo "Substituting header symbols"
find $PATHS -type f -exec sed -i -E 's/#include <mdv\/(.*)>/#include "mdv\/\1"/g' {} +
echo "Running clang format on .hpp files"
find $PATHS -type f -name \*.hpp -exec clang-format -i {} +
echo "Running clang format on .cpp files"
find $PATHS -type f -name \*.cpp -exec clang-format -i {} +
