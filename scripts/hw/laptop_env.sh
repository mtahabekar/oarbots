# Source this on the compute laptop to join a fabric hardware session:
#   source scripts/hw/laptop_env.sh                  # session domain
#   FABRIC_DOMAIN=0 source scripts/hw/laptop_env.sh  # watch the stock system instead
# Kept separate from the simulation rehearsal, which uses Fast DDS on domain 42.
_fabric_here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_fabric_here/lib.sh"
_fabric_picked="$(fabric_pick_address)"
if [ -z "$_fabric_picked" ]; then
    echo "No ${FABRIC_SUBNET_PREFIX}x address. Give the wired adapter one, for example:" >&2
    echo "  nmcli device modify <wired interface> +ipv4.addresses ${FABRIC_SUBNET_PREFIX}200/24" >&2
else
    read -r _fabric_iface _fabric_address <<< "$_fabric_picked"
    fabric_write_cyclone_profile "$HOME/.cache/fabric_hw/cyclonedds.xml" "$_fabric_address"
    source /opt/ros/jazzy/setup.bash
    _fabric_install="$(cd -- "$_fabric_here/../../../.." && pwd)/install/setup.bash"
    [ -f "$_fabric_install" ] && source "$_fabric_install"
    fabric_export_ros_env "$HOME/.cache/fabric_hw/cyclonedds.xml"
    echo "ROS 2 on $_fabric_iface $_fabric_address, CycloneDDS, domain $ROS_DOMAIN_ID"
fi
unset _fabric_here _fabric_picked _fabric_iface _fabric_address _fabric_install
