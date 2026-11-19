#!/usr/bin/env bash
# Bring the fabric cell up for a test day and check everything, from the compute laptop (repo root):
#   scripts/hw/cell_up.sh            # bring up and check
#   scripts/hw/cell_up.sh --check    # check only: start nothing, stop nothing
#   scripts/hw/cell_up.sh --deploy   # also update ~/fabric_ws on every machine to the fork's branch head
# Asks once for the lab password (scripts/hw/lab_ssh.sh). Steps, each reported OK / WARN / FAIL:
#   1 laptop lab address   2 hosts (ping, ssh)   3 clocks   4 deployed code   5 stock services and sessions
#   6 streams on the session domain   7 camera frame age on each camera host   8 where the arms are
#   9 what to run next (scripts/hw/fabric_run.sh)
# Physical prerequisites (not checkable from here): robots, helpers, NUC, tablet powered; arms powered and
# not in fault; Blue's Rokubi USB on Silver's riser hub (ground loop, lab log 2026-11-18); camera cables slack.
# The whole script is one block, so bash reads all of it before running any: editing this file while a run
# from it is still going (2026-11-19 16:13: a go then executed part of deploy-helpers) cannot change that run.
{
set -uo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$here/../.." && pwd)"
cd "$repo"
check_only=0; deploy=0
for a in "$@"; do case "$a" in --check) check_only=1 ;; --deploy) deploy=1 ;; *) sed -n '2,13p' "$0"; exit 2 ;; esac; done
# shellcheck source=lab_ssh.sh
source "$here/lab_ssh.sh"; lab_ssh_setup
fails=0; warns=0
ok()   { printf '  \033[32mOK\033[0m    %s\n' "$*"; }
warn() { printf '  \033[33mWARN\033[0m  %s\n' "$*"; warns=$((warns+1)); }
fail() { printf '  \033[31mFAIL\033[0m  %s\n' "$*"; fails=$((fails+1)); }
step() { printf '\n== %s\n' "$*"; }

IFACE=enx3c8cf8eb0ec5; LAB_ADDR=192.168.1.200
declare -A SSH_TARGET=([blue]=oarbot_blue@192.168.1.71 [silver]=oarbot_silver@192.168.1.110 [nuc]=nuc@192.168.1.90 [tablet]=tablet@192.168.1.99)
declare -A STOCK=([blue]=oarbot_blue_helper_laptop_launch.service [silver]=oarbot_silver_helper_laptop_launch.service [nuc]=nuc_launch.service)

step "1 Laptop on the lab network ($IFACE)"
if ! ip link show "$IFACE" >/dev/null 2>&1; then
    fail "USB Ethernet $IFACE not present: plug the adapter into the switch"
else
    [ "$(cat /sys/class/net/$IFACE/operstate 2>/dev/null)" = up ] && ok "link up, $(cat /sys/class/net/$IFACE/speed 2>/dev/null) Mb/s" || fail "link down: check the cable to the switch"
    if ip -4 addr show "$IFACE" | grep -q "$LAB_ADDR/"; then ok "$LAB_ADDR present"
    elif [ $check_only = 1 ]; then fail "$LAB_ADDR missing: nmcli device modify $IFACE +ipv4.addresses $LAB_ADDR/24"
    else nmcli device modify "$IFACE" +ipv4.addresses "$LAB_ADDR/24" >/dev/null && sleep 2 && ok "$LAB_ADDR re-added (it is lost at every replug)" || fail "could not add $LAB_ADDR"; fi
fi

if mountpoint -q /media/taha/ExtermeLinux; then ok "simulation disk mounted (/media/taha/ExtermeLinux: Isaac Sim, oarbot_simulation)"
else warn "simulation disk not mounted (/media/taha/ExtermeLinux): hardware runs do not need it; the simulated cell, Isaac Sim and oarbot_simulation do. Plug it in (or: udisksctl mount -b /dev/disk/by-label/ExtermeLinux)"; fi

step "2 Hosts"
for h in 192.168.1.1:router 192.168.1.71:blue-helper-wired 192.168.1.102:blue-helper-wifi 192.168.1.110:silver-helper-wired \
         192.168.1.104:silver-helper-wifi 192.168.1.90:nuc 192.168.1.99:tablet 192.168.1.101:blue-dingo 192.168.1.103:silver-dingo; do
    ping -c1 -W1 "${h%%:*}" >/dev/null 2>&1 && ok "${h#*:} ${h%%:*}" || { case "${h#*:}" in *dingo*|*wifi*) warn "${h#*:} ${h%%:*} not answering" ;; *) fail "${h#*:} ${h%%:*} not answering" ;; esac; }
done
for m in blue silver nuc tablet; do
    name="$("$LAB_SSH" "${SSH_TARGET[$m]}" hostname 2>/dev/null)" && ok "ssh $m ($name)" || fail "ssh $m (${SSH_TARGET[$m]})"
done

step "3 Clocks against the laptop (target under 5 ms; the laptop's chrony serves the LAN)"
python3 "$here/clock_offsets.py" "${SSH_TARGET[blue]}" "${SSH_TARGET[silver]}" "${SSH_TARGET[nuc]}" "${SSH_TARGET[tablet]}" 2>/dev/null |
while read -r host off unit _; do
    [ "$unit" = ms ] || { warn "$host: no offset ($off $unit)"; continue; }
    a="${off#[+-]}"; limit=5; [[ "$host" == tablet* ]] && limit=10
    awk -v a="$a" -v l="$limit" 'BEGIN{exit !(a<=l)}' && ok "$host $off ms" || warn "$host $off ms (over $limit ms: wait for timesyncd, or check its server)"
done

step "4 Deployed code (~/fabric_ws on each machine against the fork's jazzy-platform-hardening)"
git fetch -q fork 2>/dev/null; head="$(git rev-parse --short fork/jazzy-platform-hardening 2>/dev/null)"
[ -n "$(git status --porcelain --untracked-files=no)" ] && warn "laptop checkout has uncommitted changes (the robots run committed code only)"
for m in blue silver nuc tablet; do
    dep="$("$LAB_SSH" "${SSH_TARGET[$m]}" 'sed -n "s/^commit //p" ~/fabric_ws/DEPLOYED 2>/dev/null' | cut -c1-7)"
    co="$("$LAB_SSH" "${SSH_TARGET[$m]}" 'git -C ~/fabric_ws rev-parse --short=7 HEAD 2>/dev/null')"
    if [ "$dep" = "$head" ]; then ok "$m built at $dep"
    elif [ $deploy = 1 ] && [ $check_only = 0 ]; then
        echo "  deploying $head to $m (session stopped first; arms hold their pose)"
        scripts/hw/lab.sh session "$m" stop >/dev/null 2>&1
        scripts/hw/lab.sh setup "$m" 2>&1 | tail -1 | sed 's/^/    /'
    elif [ "$co" = "$head" ]; then warn "$m built at ${dep:-?}, scripts/config checked out at $co (packages changed since the build? scripts/hw/lab.sh setup $m)"
    else warn "$m built at ${dep:-?}, fork at $head (rerun with --deploy, or scripts/hw/lab.sh setup $m)"; fi
done

step "5 Stock services stopped, fabric sessions running (domain 17)"
for m in nuc blue silver tablet; do
    unit="${STOCK[$m]:-}"
    if [ -n "$unit" ]; then
        state="$("$LAB_SSH" "${SSH_TARGET[$m]}" "systemctl is-active $unit" 2>/dev/null)"
        if [ "$state" = active ]; then
            if [ $check_only = 1 ]; then warn "$m stock $unit is running (the session needs it stopped)"
            else lab_sudo "${SSH_TARGET[$m]}" systemctl stop "$unit" >/dev/null 2>&1 && ok "$m stock $unit stopped (not disabled: a reboot restores it)" || fail "$m: could not stop $unit"; fi
        else ok "$m stock $unit not running"; fi
    fi
    status="$(scripts/hw/lab.sh session "$m" status 2>/dev/null)"
    if grep -q '^fabric-stack: active' <<< "$status"; then ok "$m session running"
    elif [ $check_only = 1 ]; then warn "$m session not running"
    else
        [ "$m" = tablet ] && echo "  (tablet: close the stock tablet GUI first; only one GUI can hold the deadman Arduino)"
        scripts/hw/lab.sh session "$m" start 2>&1 | grep -E 'started|rror' | sed 's/^/    /'
        started_any=1
    fi
done
[ "${started_any:-0}" = 1 ] && { echo "  waiting 30 s for drivers, cameras and MoveIt"; sleep 30; }

step "6 Streams on the session domain (measured from the laptop, 8 s; no images over the network)"
set +u  # the ROS setup scripts read unset variables
# shellcheck disable=SC1091
source "$here/laptop_env.sh" >/dev/null 2>&1
python3 - <<'EOF'
import time, rclpy, collections
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SD
from sensor_msgs.msg import JointState, CameraInfo
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray
rclpy.init(); n = Node('cell_up_streams'); c = collections.Counter(); guard = {}
want = {}
def sub(t, typ, name, rate, qos=10):
    want[name] = rate; n.create_subscription(typ, t, lambda m: c.update([name]), qos)
for r in ('blue', 'silver'):
    sub(f'/oarbot_{r}/joint_states', JointState, f'{r} arm joint states', 50)
    sub(f'/oarbot_{r}/force_torque_raw', WrenchStamped, f'{r} F/T raw', 97, SD)
    sub(f'/oarbot_{r}/force_torque_calibrated', WrenchStamped, f'{r} F/T calibrated', 80, SD)
    sub(f'/oarbot_{r}/azure_kinect/rgb/camera_info', CameraInfo, f'{r} wrist camera info', 15, SD)
    n.create_subscription(DiagnosticArray, f'/oarbot_{r}/diagnostics', lambda m, r=r: [guard.__setitem__(r, s.message) for s in m.status if s.name.endswith('command_guard')], 10)
sub('/nuc/rgb/camera_info', CameraInfo, 'overhead camera info', 15, SD)
sub('/nuc/preview/rgb/image_raw', __import__('sensor_msgs.msg', fromlist=['Image']).Image, 'overhead preview (640x360)', 5, SD)
sub('/tablet_gui/deadman', Bool, 'tablet deadman', 50)
sub('/tablet_gui/e_stop', Bool, 'tablet e-stop', 50)
end = time.time() + 8
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
for name, rate in want.items():
    hz = c[name] / 8
    tag = 'OK  ' if hz >= 0.8 * rate else ('WARN' if hz > 0 else 'FAIL')
    col = {'OK  ': '32', 'WARN': '33', 'FAIL': '31'}[tag]
    print(f'  \033[{col}m{tag}\033[0m  {name}: {hz:.1f} Hz (expected ~{rate})')
for r in ('blue', 'silver'):
    print(f'  {"OK  " if r in guard else "FAIL"}  {r} command guard: {guard.get(r, "no diagnostics")}')
print('  (the laptop can take up to a minute to discover new nodes: rerun --check if a stream reads 0)')
EOF

step "7 Camera frame age on each camera's own host (want under 0.1 s; a stale camera needs a restart)"
for cam in nuc:/nuc blue:/oarbot_blue/azure_kinect silver:/oarbot_silver/azure_kinect; do
    line="$(scripts/hw/lab.sh session "${cam%%:*}" exec python3 fabric_ws/scripts/hw/frame_age.py "${cam#*:}" 4 2>/dev/null | grep '^image')"
    med="$(sed -n 's/.*median\/max [+-]*[0-9.]* \/ +\([0-9.]*\).*/\1/p' <<< "$line")"
    if [ -z "$med" ]; then fail "${cam%%:*} camera: no images (driver exited? a wrist camera USB drops when its cable strains during arm motion). Restart: scripts/hw/lab.sh session ${cam%%:*} stop; scripts/hw/lab.sh session ${cam%%:*} start"
    elif awk -v m="$med" 'BEGIN{exit !(m<0.1)}'; then ok "${cam%%:*} camera image age ${med} s"
    else warn "${cam%%:*} camera image age ${med} s: stale. Restart it: scripts/hw/lab.sh session ${cam%%:*} stop; scripts/hw/lab.sh session ${cam%%:*} start"; fi
done

step "8 Arms"
python3 - <<'EOF'
import time, math, yaml, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
grasp = yaml.safe_load(open('experiments/hardware/cell/grasp.yaml'))
rclpy.init(); n = Node('cell_up_arms'); q = {}
for r in ('blue', 'silver'):
    n.create_subscription(JointState, f'/oarbot_{r}/joint_states', lambda m, r=r: q.__setitem__(r, list(m.position)), 10)
end = time.time() + 4
while time.time() < end and len(q) < 2: rclpy.spin_once(n, timeout_sec=0.05)
for r in ('blue', 'silver'):
    if r not in q:
        print(f'  FAIL  {r}: no joint states'); continue
    d = [math.degrees(abs(math.atan2(math.sin(a-b), math.cos(a-b)))) for a, b in zip(q[r][:6], grasp[r])]
    fingers = ' fingers ' + ('closed' if max(q[r][6:9]) > 1.0 else 'open') if len(q[r]) >= 9 else ''
    where = 'at the grasp pose' if max(d) < 3 else f'{max(d):.0f} deg from the grasp pose (move with fabric_run.sh recentre)'
    print(f'  {"OK  " if max(d) < 3 else "WARN"}  {r}: {where};{fingers}; joints {[round(v, 3) for v in q[r][:6]]}')
EOF

step "Summary"
echo "  failures: $fails, warnings: $warns (stream and arm lines above are counted by eye)"
cat <<'EOF'

Next (scripts/hw/fabric_run.sh; procedure: experiments/hardware/sessions/2026-11-18_s5_closed_loop/README.md):
  scripts/hw/fabric_run.sh recentre            # only if an arm is away from its grasp pose (deadman, ~5 s)
  scripts/hw/fabric_run.sh perception 3600     # perception on all three cameras, workspace profile, 1 h
  scripts/hw/fabric_run.sh bridges start       # joint-mode CBF bridges, disarmed
  scripts/hw/fabric_run.sh twin                # estimator + twin + RViz (commands off); new run folder; keep open
  scripts/hw/fabric_run.sh status              # second terminal: go/no-go readout (twin following, hands seen)
  scripts/hw/fabric_run.sh enable              # commands on + bridges armed; then the deadman (release, press, hold)
  scripts/hw/fabric_run.sh disable             # commands off + bridges disarmed
  Ctrl+C in the twin terminal, then: scripts/hw/fabric_run.sh save <run folder>
EOF
exit
}
