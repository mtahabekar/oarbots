#!/usr/bin/env bash
# Explicit maintenance operation. The runtime service must already be stopped.
set -eo pipefail
workspace_dir="${1:?Usage: update_workspace.sh /path/to/repository runtime.service}"
runtime_unit="${2:?Supply the runtime service name}"
if systemctl is-active --quiet "$runtime_unit"; then
    echo "Refusing to rebuild while $runtime_unit is running. Stop it during supervised maintenance." >&2
    exit 1
fi
cd "$workspace_dir"
git diff --quiet
git diff --cached --quiet
git pull --ff-only
source /opt/ros/jazzy/setup.bash
vcs import src < packages.repos
# Existing dependencies at another revision are rejected, never silently reset.
python3 scripts/apply_dependency_patches.py --workspace-src "$workspace_dir/src"
colcon build
source install/setup.bash
# Runtime tests run in an isolated localhost-only ROS domain.
bash scripts/check_platform.sh
echo 'Build and platform checks passed. Install updated service/launch scripts separately, then perform hardware acceptance.'
