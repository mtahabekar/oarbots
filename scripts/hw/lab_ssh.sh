# Source this (bash): sets LAB_SSH to a password-based ssh wrapper for the lab hosts.
# The lab machines have no keys installed (shared machines). Passwords (docusaurus/docs/getting-started/
# device-credentials.mdx; user, 2026-11-21: 1234 on every lab device we use) are built in, so no script asks:
# LAB_PASSWORD (helpers, NUC, tablet; default 1234) and LAB_DINGO_PASSWORD (the Dingos' `robot` account; default
# from the credentials doc) override them. The wrapper files are rewritten at every call (a stale or empty cache,
# 2026-11-21, made every ssh fail) in $XDG_RUNTIME_DIR/fabric_lab (tmpfs, mode 700).
lab_ssh_setup() {
    local dir="${XDG_RUNTIME_DIR:-/tmp/fabric_lab_$UID}/fabric_lab"
    local pw="${LAB_PASSWORD:-1234}" dpw="${LAB_DINGO_PASSWORD:-clearpath}"
    mkdir -p "$dir" && chmod 700 "$dir"
    (
        umask 077
        printf '%s\n' "$pw" > "$dir/pw"
        printf '%s\n' "$dpw" > "$dir/dpw"
        cat > "$dir/askpass" <<EOF
#!/bin/sh
case "\$1" in *robot@*) cat "$dir/dpw" ;; *) cat "$dir/pw" ;; esac
EOF
        cat > "$dir/lssh" <<EOF
#!/bin/sh
SSH_ASKPASS="$dir/askpass" SSH_ASKPASS_REQUIRE=force DISPLAY=\${DISPLAY:-:0} exec ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new -o PubkeyAuthentication=no "\$@"
EOF
        chmod 700 "$dir/askpass" "$dir/lssh"
    )
    export LAB_SSH="$dir/lssh"
    LAB_PW_FILE="$dir/pw"
}

# sudo on a lab host without a terminal: the password goes over ssh's stdin, never on a command line.
lab_sudo() {  # lab_sudo <user@host> <command...>
    local target="$1"; shift
    "$LAB_SSH" "$target" "sudo -S -p '' $*" < "$LAB_PW_FILE"
}
