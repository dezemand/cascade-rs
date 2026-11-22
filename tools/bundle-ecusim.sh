#!/bin/sh
# Wrap ecusim-gui (and the ecusim server it can start) in a minimal macOS app
# bundle: target/EcuSim.app
# Usage: tools/bundle-ecusim.sh [debug|release]   (default: release)
#        open target/EcuSim.app --args --rom path/to/simk41-no-immo.bin
set -eu
profile=${1:-release}
here=$(cd "$(dirname "$0")/.." && pwd)
if [ "$profile" = release ]; then
  cargo build -q --release -p ecusim-gui -p ecusim
else
  cargo build -q -p ecusim-gui -p ecusim
fi
app="$here/target/EcuSim.app"
rm -rf "$app"
mkdir -p "$app/Contents/MacOS"
cp "$here/target/$profile/ecusim-gui" "$here/target/$profile/ecusim" "$app/Contents/MacOS/"
cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleName</key><string>EcuSim</string>
  <key>CFBundleIdentifier</key><string>dev.cascade-rs.ecusim</string>
  <key>CFBundleExecutable</key><string>ecusim-gui</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>0.1.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>NSAppSleepDisabled</key><true/>
</dict></plist>
PLIST
echo "$app"
