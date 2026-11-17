#!/usr/bin/env bash
# Double-click on the tablet (right-click, Run as a Program) to open the fabric-session GUI.
# Close the stock GUI first. Closing this GUI's window ends it; or run: fabric_session.sh stop
exec bash "$(dirname -- "$(readlink -f -- "$0")")/fabric_session.sh" start
