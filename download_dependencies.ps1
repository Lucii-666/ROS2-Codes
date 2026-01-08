# PowerShell script to download dependencies
if (-not (Test-Path "src")) {
    New-Item -ItemType Directory -Path "src"
}

Set-Location src

if (Test-Path "slam_toolbox") {
    Write-Host "slam_toolbox already exists."
} else {
    Write-Host "Cloning slam_toolbox (Humble branch)..."
    git clone -b humble https://github.com/SteveMacenski/slam_toolbox.git
}

Write-Host "Dependencies downloaded."
Set-Location ..
