# Source this (bash): sets LAB_SSH to a password-based ssh wrapper for the lab hosts.
# The lab machines have no keys installed (shared machines); their passwords are in
# docusaurus/docs/getting-started/device-credentials.mdx. The password is asked once per login session
# (or taken from LAB_PASSWORD; LAB_DINGO_PASSWORD for the Dingos' `robot` account) and kept only in
# $XDG_RUNTIME_DIR/fabric_lab (tmpfs, mode 700), never in the repository.
lab_ssh_setup() {
    local dir="${XDG_RUNTIME_DIR:-/tmp/fabric_lab_$UID}/fabric_lab"
    if [ ! -x "$dir/lssh" ] || [ ! -s "$dir/pw" ]; then
        mkdir -p "$dir" && chmod 700 "$dir"
        local pw="${LAB_PASSWORD:-}" dpw="${LAB_DINGO_PASSWORD:-}"
        if [ -z "$pw" ]; then read -r -s -p "Lab password (helpers, NUC, tablet): " pw; echo; fi
        if [ -z "$dpw" ]; then read -r -s -p "Dingo password (robot@, Enter to skip): " dpw; echo; fi
        umask 077
        printf '%s\n' "$pw" > "$dir/pw"
        printf '%s\n' "${dpw:-$pw}" > "$dir/dpw"
        cat > "$dir/askpass" <<EOF
#!/bin/sh
case "\$1" in *robot@*) cat "$dir/dpw" ;; *) cat "$dir/pw" ;; esac
EOF
        cat > "$dir/lssh" <<EOF
#!/bin/sh
SSH_ASKPASS="$dir/askpass" SSH_ASKPASS_REQUIRE=force DISPLAY=\${DISPLAY:-:0} exec ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new -o PubkeyAuthentication=no "\$@"
EOF
        chmod 700 "$dir/askpass" "$dir/lssh"
    fi
    export LAB_SSH="$dir/lssh"
    LAB_PW_FILE="$dir/pw"
}

# sudo on a lab host without a terminal: the password goes over ssh's stdin, never on a command line.
lab_sudo() {  # lab_sudo <user@host> <command...>
    local target="$1"; shift
    "$LAB_SSH" "$target" "sudo -S -p '' $*" < "$LAB_PW_FILE"
}
