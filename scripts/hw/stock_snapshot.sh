#!/usr/bin/env bash
# Print the stock OARBot system's state on this machine in a stable form, for diffing.
# Read-only and needs no sudo. Take one before the first fabric session (the baseline) and one
# after every session; outside a session the two must be identical.
#   From the laptop: scripts/hw/lab.sh snapshot <blue|silver|tablet|nuc>
set -o pipefail
short() { sha256sum | cut -c1-16; }
hashfiles() { for f in "$@"; do [ -f "$f" ] && echo "$(sha256sum < "$f" | cut -c1-16) $f"; done; }

echo "host $(hostname) user $(whoami)"

ws="$HOME/ros2_ws"
if [ -d "$ws/.git" ]; then
    echo "ros2_ws head $(git -C "$ws" rev-parse HEAD) branch $(git -C "$ws" branch --show-current)"
    echo "ros2_ws worktree $(git -C "$ws" status --porcelain | short) ($(git -C "$ws" status --porcelain | wc -l) entries)"
    for d in "$ws"/src/*/; do
        [ -d "$d.git" ] && echo "ros2_ws dep $(basename "$d") $(git -C "$d" rev-parse --short HEAD) modified=$(git -C "$d" status --porcelain | wc -l)"
    done
    # No install timestamps: the stock CD service rebuilds ~/ros2_ws from source at every boot,
    # so the source state above determines what is installed.
    [ -d "$ws/.venv" ] && echo "ros2_ws venv $("$ws/.venv/bin/pip" freeze 2>/dev/null | short)"
fi

for u in $(systemctl list-unit-files --no-legend 'oarbot*' 'nuc*' 'tablet*' 'xorg-headless*' 'clearpath*' 2>/dev/null | awk '{print $1}'); do
    echo "unit $u enabled=$(systemctl is-enabled "$u" 2>/dev/null) active=$(systemctl is-active "$u" 2>/dev/null) definition=$(systemctl cat "$u" 2>/dev/null | short)"
done
echo "user units enabled: $(systemctl --user list-unit-files --state=enabled --no-legend 2>/dev/null | awk '{print $1}' | tr '\n' ' ')"

hashfiles /usr/local/bin/* /etc/udev/rules.d/* | sed 's/^/file /'
ls -l --time-style=+%s /etc/netplan 2>/dev/null | awk 'NR > 1 {print "netplan", $1, $5, $6, $7}'
nmcli -t -f NAME,FILENAME connection show 2>/dev/null | grep ':/etc/' | sed 's/^/saved connection /'
echo "packages $(dpkg-query -W -f '${Package} ${Version}\n' | wc -l) $(dpkg-query -W -f '${Package} ${Version}\n' | short)"
echo "sysctl rmem_max=$(cat /proc/sys/net/core/rmem_max) wmem_max=$(cat /proc/sys/net/core/wmem_max)"
hashfiles "$HOME/.bashrc" "$HOME/.profile" "$HOME/tablet_launch.sh" "$HOME/.ros/cyclonedds.xml" | sed 's/^/file /'
echo "crontab $(crontab -l 2>/dev/null | short)"
