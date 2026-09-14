# Deploy to New Radxa (Debian SD Card)
# Run in PowerShell: .\deploy_to_new_radxa.ps1

$ErrorActionPreference = "Stop"

Write-Host "=======================================" -ForegroundColor Cyan
Write-Host " DEPLOY TO RADXA ZERO 3W (Debian SD)"  -ForegroundColor Cyan
Write-Host "=======================================" -ForegroundColor Cyan
Write-Host "Make sure Radxa is BOOTED and connected to WiFi."
Write-Host ""

# Ask for IP
$ip = Read-Host "Enter Radxa IP Address (e.g. 10.36.193.10)"
if (-not $ip) { Write-Error "IP Address is required!"; exit }

$user = "shreyash"
$localdir = "C:\Users\adish\.gemini\antigravity\scratch\drone_project"

Write-Host ""
Write-Host "[1/3] Copying drone project to Radxa..." -ForegroundColor Yellow
Write-Host "  From: $localdir"
Write-Host "  To:   $user@${ip}:~/drone_project/"
Write-Host "  (Enter password when prompted)"

# Copy the whole drone project (includes raxda_bridge, setup scripts, etc.)
scp -r "$localdir\raxda_bridge" "${user}@${ip}:/home/$user/drone_project/"

if ($LASTEXITCODE -ne 0) {
    Write-Error "SCP Failed! Check IP/Password."
    exit
}
Write-Host "  Files copied." -ForegroundColor Green

Write-Host ""
Write-Host "[2/3] Making setup script executable..." -ForegroundColor Yellow
ssh "$user@$ip" "chmod +x ~/drone_project/raxda_bridge/post_flash_setup_v3_debian.sh"

Write-Host ""
Write-Host "[3/3] Running setup script (needs sudo password)..." -ForegroundColor Yellow
ssh -t "$user@$ip" "sudo ~/drone_project/raxda_bridge/post_flash_setup_v3_debian.sh"

Write-Host ""
if ($LASTEXITCODE -eq 0) {
    Write-Host "=======================================" -ForegroundColor Green
    Write-Host " DEPLOYMENT COMPLETE!" -ForegroundColor Green
    Write-Host "=======================================" -ForegroundColor Green
    Write-Host ""
    Write-Host " Next: SSH in and run 'sudo tailscale up' then 'sudo reboot'"
} else {
    Write-Host " Setup finished with some errors (check output above)" -ForegroundColor Red
}
