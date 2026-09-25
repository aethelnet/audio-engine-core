#!/usr/bin/env bash
# ============================================================================
# Sovereign Aethel Desk: Standalone Linux AppImage Packaging Script
# Creates a 100% portable double-clickable AppImage executable
# ============================================================================

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build-appimage"
APP_DIR="${ROOT_DIR}/dist/AppDir"
OUTPUT_DIR="${ROOT_DIR}/dist"

echo "=== [1/4] Building Release Binary ==="
cmake -B "${BUILD_DIR}" -S "${ROOT_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DAUDIO_CORE_ENABLE_PIPEWIRE=ON \
    -DCMAKE_INSTALL_PREFIX=/usr

cmake --build "${BUILD_DIR}" --target aethel_desk -j"$(nproc)"

echo "=== [2/4] Assembling AppDir Hierarchy ==="
rm -rf "${APP_DIR}"
mkdir -p "${APP_DIR}/usr/bin"
mkdir -p "${APP_DIR}/usr/share/applications"
mkdir -p "${APP_DIR}/usr/share/icons/hicolor/256x256/apps"

cp "${BUILD_DIR}/aethel_desk" "${APP_DIR}/usr/bin/aethel_desk"

# Desktop metadata entry
cat << 'EOF' > "${APP_DIR}/aethel_desk.desktop"
[Desktop Entry]
Type=Application
Name=Aethel Audio Desk
GenericName=Digital Audio Workstation & Tracker
Comment=Mastering-Grade Hybrid Arranger, Renoise Tracker & Modular Synthesizer
Exec=aethel_desk
Icon=aethel_desk
Categories=AudioVideo;Audio;Midi;Sequencer;
Terminal=false
StartupNotify=true
EOF

cp "${APP_DIR}/aethel_desk.desktop" "${APP_DIR}/usr/share/applications/"

# Generate SVG App Icon
cat << 'EOF' > "${APP_DIR}/aethel_desk.svg"
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 256 256">
  <rect width="256" height="256" rx="48" fill="#111318"/>
  <circle cx="128" cy="128" r="96" fill="none" stroke="#2563EB" stroke-width="12"/>
  <path d="M 64 128 Q 96 48 128 128 T 192 128" fill="none" stroke="#F59E0B" stroke-width="14" stroke-linecap="round"/>
  <circle cx="128" cy="128" r="16" fill="#10B981"/>
</svg>
EOF

cp "${APP_DIR}/aethel_desk.svg" "${APP_DIR}/usr/share/icons/hicolor/256x256/apps/aethel_desk.png"
ln -sf "aethel_desk.svg" "${APP_DIR}/.DirIcon"

# AppRun Launch Script
cat << 'EOF' > "${APP_DIR}/AppRun"
#!/bin/sh
SELF=$(readlink -f "$0")
HERE=${SELF%/*}
export PATH="${HERE}/usr/bin:${PATH}"
export LD_LIBRARY_PATH="${HERE}/usr/lib:${LD_LIBRARY_PATH:-}"
exec "${HERE}/usr/bin/aethel_desk" "$@"
EOF
chmod +x "${APP_DIR}/AppRun"

echo "=== [3/4] Checking appimagetool ==="
mkdir -p "${OUTPUT_DIR}"
if ! command -v appimagetool &> /dev/null; then
    echo "Notice: appimagetool not found in PATH."
    echo "AppDir successfully prepared at: ${APP_DIR}"
    echo "To finalize .AppImage, run: appimagetool ${APP_DIR} ${OUTPUT_DIR}/AethelAudioDesk-x86_64.AppImage"
else
    echo "=== [4/4] Generating AppImage ==="
    ARCH=x86_64 appimagetool "${APP_DIR}" "${OUTPUT_DIR}/AethelAudioDesk-x86_64.AppImage"
    echo "Success: ${OUTPUT_DIR}/AethelAudioDesk-x86_64.AppImage created!"
fi
