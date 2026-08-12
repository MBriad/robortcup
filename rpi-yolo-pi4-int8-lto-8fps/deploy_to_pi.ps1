param(
    [Parameter(Mandatory = $false)]
    [string]$PiIp,
    [string]$PiUser = "pi"
)

$ErrorActionPreference = "Stop"
$packageDir = $PSScriptRoot

if ([string]::IsNullOrWhiteSpace($PiIp)) {
    $PiIp = Read-Host "Raspberry Pi IP"
}

$remote = "${PiUser}@${PiIp}"
$remoteDir = "/tmp/rpi-yolo-int8-lto-deploy"
$items = @(
    "install.sh",
    "rpi-yolo",
    "rpi-yolo-preview",
    "rpi_yolo_api.py",
    "vision_client_example.py",
    "README.txt",
    "VERSION",
    "vision_service_cpp",
    "model"
)

Write-Host "Creating remote directory: $remoteDir"
& ssh $remote "rm -rf $remoteDir && mkdir -p $remoteDir"
if ($LASTEXITCODE -ne 0) { throw "SSH connection failed." }

foreach ($item in $items) {
    Write-Host "Uploading $item"
    & scp -r (Join-Path $packageDir $item) "${remote}:${remoteDir}/"
    if ($LASTEXITCODE -ne 0) { throw "Upload failed: $item" }
}

Write-Host "Running installer"
& ssh -t $remote "cd $remoteDir && chmod +x install.sh rpi-yolo rpi-yolo-preview && sudo ./install.sh"
if ($LASTEXITCODE -ne 0) { throw "Raspberry Pi installation failed." }

Write-Host "Installed. Run: rpi-yolo --max-frames 20"
Write-Host "Preview: http://${PiIp}:8080/ after running rpi-yolo-preview"
