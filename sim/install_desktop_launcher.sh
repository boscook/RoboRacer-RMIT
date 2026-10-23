#!/usr/bin/env bash
# Installs the one-click launcher. Run once:  bash install_desktop_launcher.sh

KIT="$(cd "$(dirname "$0")" && pwd)"
DEST="$HOME/roboracer_twin"
mkdir -p "$DEST/bin"
cp "$KIT"/bin/*.sh "$DEST/bin/"
chmod +x "$DEST"/bin/*.sh
[ -f "$DEST/log.py" ] || cp "$KIT/log.py" "$DEST/log.py"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || true)"; [ -n "$DESK" ] || DESK="$HOME/Desktop"
mkdir -p "$DESK"
F="$DESK/RoboRacer Sim.desktop"
cat > "$F" <<DESKTOP
[Desktop Entry]
Type=Application
Name=RoboRacer Sim
Comment=Start the AutoDRIVE RoboRacer digital twin
Exec=$DEST/bin/roborace_start.sh
Icon=utilities-terminal
Terminal=false
DESKTOP
chmod +x "$F"
gio set "$F" metadata::trusted true 2>/dev/null || true
echo "Installed. Double-click 'RoboRacer Sim' on the desktop."
echo "If the icon shows a warning, right-click it and choose 'Allow Launching'."
