#!/usr/bin/env bash
# Install the fastviz desktop entry + icon for the *current user*, so a window
# launched from a dev build (cargo run) shows the same icon as the packaged
# .deb. Run this on the HOST, not inside the devcontainer.
#
# Why the host: on Wayland the compositor draws no client-supplied window icon;
# it resolves the icon from a .desktop file whose basename matches the window
# app_id ("fastviz", set in crates/app/src/main.rs). That lookup happens in the
# compositor, which runs on the host — so the desktop file + icon must live in
# the host's XDG data dirs, even when the binary runs in a container.
#
# Mirrors the layout the .deb installs (see crates/app/Cargo.toml [metadata.deb
# assets]); the only difference is the install prefix (~/.local/share here vs
# /usr/share for the package).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
data="${XDG_DATA_HOME:-$HOME/.local/share}"

apps="$data/applications"
icons="$data/icons/hicolor/1024x1024/apps"

mkdir -p "$apps" "$icons"
install -m 644 "$here/fastviz.desktop"                  "$apps/fastviz.desktop"
install -m 644 "$here/fastviz_round_f_square_1024.png"  "$icons/fastviz.png"

# Refresh caches if the tools are present (harmless if they aren't).
command -v update-desktop-database >/dev/null && update-desktop-database "$apps" || true
command -v gtk-update-icon-cache   >/dev/null && gtk-update-icon-cache -f -t "$data/icons/hicolor" >/dev/null 2>&1 || true

echo "Installed fastviz.desktop -> $apps"
echo "Installed icon            -> $icons/fastviz.png"
echo "You may need to log out/in (or restart the shell/compositor) for the icon to refresh."
