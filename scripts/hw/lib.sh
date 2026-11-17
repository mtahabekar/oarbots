# Shared settings for fabric hardware sessions. Sourced by the other scripts in this directory.

FABRIC_REPO_URL="${FABRIC_REPO_URL:-https://github.com/mtahabekar/oarbots.git}"
FABRIC_REF="${FABRIC_REF:-hw-tests}"
FABRIC_WS="${FABRIC_WS:-$HOME/fabric_ws}"
# The stock system uses domain 0 and the Dingos use 1 (Silver) and 2 (Blue).
# A separate domain keeps other people's tools and our session from seeing each other.
FABRIC_DOMAIN="${FABRIC_DOMAIN:-17}"
FABRIC_SUBNET_PREFIX="${FABRIC_SUBNET_PREFIX:-192.168.1.}"

# Print "<interface> <address>" for the lab subnet, preferring a wired interface (name starts with e).
fabric_pick_address() {
    ip -4 -o addr show scope global \
        | awk -v p="$FABRIC_SUBNET_PREFIX" '{split($4, a, "/")} index(a[1], p) == 1 {print ($2 ~ /^e/ ? 0 : 1), $2, a[1]}' \
        | sort -n -k1,1 -s | head -1 | cut -d' ' -f2-
}

# Write a CycloneDDS profile bound to one address. Binding by address, not interface name,
# works when an adapter carries two IPv4 addresses. The large receive buffer lets 1080p colour
# frames (about 8 MB) through; the kernel caps it at net.core.rmem_max.
fabric_write_cyclone_profile() {
    local path="$1" address="$2"
    mkdir -p "$(dirname "$path")"
    cat > "$path" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface address="$address" />
            </Interfaces>
            <AllowMulticast>spdp</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
        <Internal>
            <SocketReceiveBufferSize min="default" max="64MB" />
        </Internal>
        <Discovery>
            <EnableTopicDiscoveryEndpoints>false</EnableTopicDiscoveryEndpoints>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
        </Discovery>
    </Domain>
</CycloneDDS>
EOF
}

# Environment shared by every process in a session.
fabric_export_ros_env() {
    local profile="$1"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID="$FABRIC_DOMAIN"
    export CYCLONEDDS_URI="file://$profile"
}
