#!/bin/bash
set -euo pipefail

# ----------------------------------------------------------------------------
# source.sh : source workspace setup based on repo location
# ----------------------------------------------------------------------------

is_sourced() {
    [[ "${BASH_SOURCE[0]}" != "${0}" ]]
}

SCRIPT_FILE="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_FILE")"
SHLIB_DIR="$SCRIPT_DIR/shlib"
source "$SHLIB_DIR/paths.sh"

# Some ROS setup scripts rely on undefined vars; relax -u temporarily.
set +u
if [[ -f "$ROS_SETUP" ]]; then
    # shellcheck source=/dev/null
    source "$ROS_SETUP"
    echo "Sourced ROS: $ROS_SETUP"
else
    echo "Error: ROS setup not found at $ROS_SETUP"
    exit 1
fi

if [[ -f "$WS_SETUP" ]]; then
    # shellcheck source=/dev/null
    source "$WS_SETUP"
    echo "Successfully sourced workspace: $WS_ROOT"
else
    echo "Error: Workspace setup.bash not found at $WS_SETUP"
    exit 1
fi
set -u

if ! is_sourced; then
    echo "Note: ./source.sh runs in a subshell. Opening a new shell with this environment."
    echo "Type 'exit' to return to your previous shell."
    exec "${SHELL:-/bin/bash}" -i
fi
