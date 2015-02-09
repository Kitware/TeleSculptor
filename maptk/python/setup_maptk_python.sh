#
# Setting up build environment python bindings
#
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
export PYTHONPATH=${SCRIPT_DIR}:$PYTHONPATH
