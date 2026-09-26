# Common paths of the shell scripts (sourced): the repository root is the parent of scripts/.
#   S     scripts directory
#   ROOT  repository root (logs/, results/ live here); exported as IJC_ROOT for the Gazebo plugins,
#         which resolve relative log paths of the worlds against it
#   PX4   PX4-Autopilot tree: $PX4_DIR if set, else <root>/PX4-Autopilot (created by setup.sh)
S=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(dirname "$S")
PX4=${PX4_DIR:-$ROOT/PX4-Autopilot}
export IJC_ROOT=$ROOT
