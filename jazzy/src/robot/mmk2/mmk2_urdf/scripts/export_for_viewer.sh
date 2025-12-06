#!/bin/bash
# Export MMK2 URDF for online viewer (viewer.robotsfan.com etc.)
# This script generates a URDF with relative paths and packages all meshes

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
EXPORT_DIR="/tmp/mmk2_export"

echo "=== MMK2 URDF Export Script ==="
echo "Package dir: $PKG_DIR"
echo "Export dir: $EXPORT_DIR"

# Clean and create export directory
rm -rf "$EXPORT_DIR"
mkdir -p "$EXPORT_DIR"

# Copy URDF and convert paths to relative
echo "Converting URDF paths..."
cp "$PKG_DIR/urdf/mmk2.urdf" "$EXPORT_DIR/mmk2.urdf"
sed -i 's|package://mmk2_urdf/||g' "$EXPORT_DIR/mmk2.urdf"

# Copy meshes
echo "Copying mesh files..."
cp -r "$PKG_DIR/meshes" "$EXPORT_DIR/"

# Create zip file
echo "Creating zip archive..."
cd "$EXPORT_DIR"
zip -r mmk2_model.zip mmk2.urdf meshes/ > /dev/null

echo ""
echo "=== Export Complete ==="
echo "Files exported to: $EXPORT_DIR"
echo ""
echo "Contents:"
ls -la "$EXPORT_DIR"
echo ""
echo "To use with online viewer:"
echo "1. Upload mmk2_model.zip to viewer.robotsfan.com"
echo "   OR"
echo "2. Upload mmk2.urdf and meshes/ folder separately"
echo ""
echo "Zip file: $EXPORT_DIR/mmk2_model.zip"
