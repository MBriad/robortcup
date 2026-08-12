#!/bin/sh
set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "Run this installer with: sudo ./install.sh" >&2
    exit 1
fi

PACKAGE_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
INSTALL_DIR=/opt/rpi-yolo

if [ "$(uname -m)" != "armv7l" ]; then
    echo "Unsupported architecture: $(uname -m); expected armv7l." >&2
    exit 1
fi

if [ ! -r /etc/os-release ]; then
    echo "Cannot read /etc/os-release." >&2
    exit 1
fi
. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "buster" ] && [ "${VERSION_ID:-}" != "10" ]; then
    echo "Unsupported OS: ${PRETTY_NAME:-unknown}; expected Raspbian Buster 10." >&2
    exit 1
fi

for required in \
    "$PACKAGE_DIR/vision_service_cpp" \
    "$PACKAGE_DIR/model/model.ncnn.param" \
    "$PACKAGE_DIR/model/model.ncnn.bin" \
    "$PACKAGE_DIR/rpi_yolo_api.py" \
    "$PACKAGE_DIR/rpi-yolo" \
    "$PACKAGE_DIR/rpi-yolo-preview"; do
    if [ ! -f "$required" ]; then
        echo "Package file missing: $required" >&2
        exit 1
    fi
done

if [ -f "$PACKAGE_DIR/SHA256SUMS.txt" ] && command -v sha256sum >/dev/null 2>&1; then
    echo "[0/4] Verifying package files"
    (cd "$PACKAGE_DIR" && sha256sum -c SHA256SUMS.txt)
fi

if ldd "$PACKAGE_DIR/vision_service_cpp" 2>/dev/null | grep -q "not found"; then
    echo "[1/4] Installing runtime libraries"
    cat > /etc/apt/sources.list.d/rpi-yolo-buster-archive.list <<'EOF'
deb [trusted=yes] http://archive.debian.org/debian buster main
EOF
    cat > /etc/apt/apt.conf.d/99-rpi-yolo-buster-archive <<'EOF'
Acquire::Check-Valid-Until "false";
EOF
    apt-get update \
        -o Dir::Etc::sourcelist="sources.list.d/rpi-yolo-buster-archive.list" \
        -o Dir::Etc::sourceparts="-" \
        -o APT::Get::List-Cleanup="0" \
        -o Acquire::AllowInsecureRepositories="true"
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        libopencv-core3.2 \
        libopencv-imgproc3.2 \
        libjpeg62-turbo \
        libgomp1 \
        libatomic1 \
        libtbb2
else
    echo "[1/4] Runtime libraries already available"
fi

echo "[2/4] Installing NCNN service and INT8 model"
install -d -m 0755 "$INSTALL_DIR/model"
install -m 0755 "$PACKAGE_DIR/vision_service_cpp" "$INSTALL_DIR/vision_service_cpp"
install -m 0644 "$PACKAGE_DIR/model/model.ncnn.param" "$INSTALL_DIR/model/model.ncnn.param"
install -m 0644 "$PACKAGE_DIR/model/model.ncnn.bin" "$INSTALL_DIR/model/model.ncnn.bin"
install -m 0644 "$PACKAGE_DIR/VERSION" "$INSTALL_DIR/VERSION"

echo "[3/4] Installing commands and Python API"
install -m 0755 "$PACKAGE_DIR/rpi-yolo" /usr/local/bin/rpi-yolo
install -m 0755 "$PACKAGE_DIR/rpi-yolo-preview" /usr/local/bin/rpi-yolo-preview
PYTHON_SITE=$(python3 -c 'import site; print(site.getsitepackages()[0])')
install -d -m 0755 "$PYTHON_SITE"
install -m 0644 "$PACKAGE_DIR/rpi_yolo_api.py" "$PYTHON_SITE/rpi_yolo_api.py"

echo "[4/4] Verifying installation"
/usr/local/bin/rpi-yolo --help >/dev/null
python3 -c 'from rpi_yolo_api import VisionClient' >/dev/null

echo
echo "Installed: $(cat "$INSTALL_DIR/VERSION")"
echo "Camera test:       rpi-yolo --max-frames 20"
echo "Browser preview:   rpi-yolo-preview"
echo "Preview URL:       http://$(hostname -I 2>/dev/null | awk '{print $1}'):8080/"
