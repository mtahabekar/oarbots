#!/usr/bin/env bash
# Create or update the fabric test workspace (~/fabric_ws) on a lab machine and build it.
# Runs as the lab account without sudo. Never reads from or writes to the stock ~/ros2_ws,
# its services, /usr/local/bin, udev rules or system packages.
#   From the laptop: scripts/hw/lab.sh setup <blue|silver|tablet|nuc> [git ref]
#   On the machine:   bash setup_fabric_ws.sh <helper|tablet|camera> [git ref]
set -eo pipefail
role="${1:?usage: setup_fabric_ws.sh <helper|tablet|camera> [ref]}"
ref="${2:-${FABRIC_REF:-jazzy-platform-hardening}}"
repo="${FABRIC_REPO_URL:-https://github.com/mtahabekar/oarbots.git}"
ws="${FABRIC_WS:-$HOME/fabric_ws}"
pyside_version="${FABRIC_PYSIDE_VERSION:-6.11.1}"   # same as the stock tablet GUI's venv

case "$(realpath -m "$ws")" in
    "$(realpath -m "$HOME/ros2_ws")"*) echo "Refusing to use the stock workspace $ws" >&2; exit 2 ;;
esac

[ -d "$ws/.git" ] || git clone --quiet "$repo" "$ws"
cd "$ws"
if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "$ws has uncommitted changes; leaving it as it is" >&2
    exit 1
fi
git fetch --quiet --tags --force --prune origin
if git rev-parse --verify --quiet "origin/$ref^{commit}" > /dev/null; then target="origin/$ref"; else target="$ref"; fi
git checkout --quiet --detach "$target"

# Machine-local files stay out of git status.
for entry in .venv/ run/ DEPLOYED OWNER.txt; do
    grep -qxF "$entry" .git/info/exclude || echo "$entry" >> .git/info/exclude
done
cat > OWNER.txt <<EOF
Fabric hardware-test workspace${FABRIC_OWNER:+ of $FABRIC_OWNER}.
It is separate from ~/ros2_ws and nothing here starts at boot. It runs only through
scripts/hw/fabric_session.sh while the stock service is stopped. See scripts/hw/README.md.
Please ask before deleting it.
EOF

mkdir -p log run
logfile="$ws/log/setup_$(date +%Y%m%d_%H%M%S).log"
echo "Building $(git describe --tags --always) for role $role; log: $logfile"
source /opt/ros/jazzy/setup.bash
# errexit does not apply inside a function whose status is tested, so every step returns explicitly.
steps() {
    vcs import --input packages.repos src || return
    python3 scripts/apply_dependency_patches.py --workspace-src "$ws/src" || return
    export MAKEFLAGS="-j4"
    case "$role" in
        helper)
            nice -n 10 colcon build --parallel-workers 4 --event-handlers console_cohesion+ || return ;;
        tablet)
            nice -n 10 colcon build --parallel-workers 2 --event-handlers console_cohesion+ \
                --packages-select kinova_msgs tablet_arduino_publisher tablet_gui || return
            # oarbot_launch is not built here: colcon would require every robot driver it lists as a
            # run dependency. fabric_session.sh launches its tablet_gui.launch.py by path instead.
            [ -x .venv/bin/python3 ] || python3 -m venv --system-site-packages .venv || return
            .venv/bin/pip install --quiet --disable-pip-version-check "PySide6==$pyside_version" || return ;;
        camera)
            # The NUC: the overhead camera driver and the camera-side perception only (the estimator,
            # twin and simulator stay on the compute laptop). Launch files run by path.
            nice -n 10 colcon build --parallel-workers 4 --event-handlers console_cohesion+ \
                --packages-up-to azure_kinect_ros_driver fabric_perception || return ;;
        *) echo "unknown role $role"; return 2 ;;
    esac
}
if ! steps > "$logfile" 2>&1; then
    tail -40 "$logfile"
    echo "Setup failed; full log: $logfile" >&2
    exit 1
fi

{
    echo "commit $(git rev-parse HEAD)"
    echo "describe $(git describe --tags --always)"
    echo "role $role"
    echo "built $(date -Is)"
} > DEPLOYED
grep -E "^Summary|packages? (failed|had stderr)" "$logfile" || true
echo "Deployed: $(tr '\n' ' ' < DEPLOYED)"
