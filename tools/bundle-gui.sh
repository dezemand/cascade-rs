#!/bin/sh
# Wrap the cascade-gui binary in a minimal macOS app bundle: target/Cascade.app
# Usage: tools/bundle-gui.sh [debug|release]   (default: release)
set -eu
profile=${1:-release}
here=$(cd "$(dirname "$0")/.." && pwd)
if [ "$profile" = release ]; then cargo build -q --release -p cascade-gui; else cargo build -q -p cascade-gui; fi
app="$here/target/Cascade.app"
rm -rf "$app"
mkdir -p "$app/Contents/MacOS"
cp "$here/target/$profile/cascade-gui" "$app/Contents/MacOS/cascade-gui"
cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleName</key><string>Cascade</string>
  <key>CFBundleIdentifier</key><string>dev.cascade-rs.gui</string>
  <key>CFBundleExecutable</key><string>cascade-gui</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>0.1.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>NSAppSleepDisabled</key><true/>
</dict></plist>
PLIST
echo "$app"
