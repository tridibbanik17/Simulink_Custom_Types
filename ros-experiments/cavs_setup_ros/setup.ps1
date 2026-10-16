# One-step setup for the ROS 2 dev container on Windows.
# Installs WSL, Docker Desktop and VS Code if they are missing, adds the
# Dev Containers extension, then opens this folder inside the container.
# Safe to run again: steps that are already done are skipped.
# Run it by double-clicking setup.bat.
$ErrorActionPreference = 'Stop'

$RepoDir = $PSScriptRoot
$Extension = 'ms-vscode-remote.remote-containers'
# Must match "workspaceFolder" in .devcontainer/devcontainer.json
$ContainerFolder = '/workspace'
$DockerExe = "$env:ProgramFiles\Docker\Docker\Docker Desktop.exe"

function Info($msg) { Write-Host "`n==> $msg" -ForegroundColor Cyan }
function Fail($msg) { Write-Host "`nERROR: $msg" -ForegroundColor Red; exit 1 }

function Refresh-Path {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
}

function Find-Code {
    $cmd = Get-Command code -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    foreach ($p in "$env:LOCALAPPDATA\Programs\Microsoft VS Code\bin\code.cmd",
                   "$env:ProgramFiles\Microsoft VS Code\bin\code.cmd") {
        if (Test-Path $p) { return $p }
    }
}

# Runs a program with its output hidden and returns whether it succeeded.
# Windows PowerShell turns a program's redirected error output into a script
# error, which 'Stop' would abort on, so relax that just for this call.
function Test-Run([scriptblock]$Command) {
    $ErrorActionPreference = 'Continue'
    $global:LASTEXITCODE = 1
    & $Command *> $null
    return $global:LASTEXITCODE -eq 0
}

function Test-Docker {
    $docker = Get-Command docker -ErrorAction SilentlyContinue
    if (-not $docker) { return $false }
    # 'docker info' can hang when Docker Desktop is stuck starting, so give up after 15 seconds
    $p = Start-Process $docker.Source -ArgumentList 'info' -NoNewWindow -PassThru `
        -RedirectStandardOutput "$env:TEMP\docker-info-out.txt" -RedirectStandardError "$env:TEMP\docker-info-err.txt"
    $null = $p.Handle  # needed for ExitCode to be available later
    if (-not $p.WaitForExit(15000)) {
        try { $p.Kill() } catch { }
        return $false
    }
    return $p.ExitCode -eq 0
}

function Install-Wsl {
    Info 'Installing WSL 2 (click Yes when Windows asks for permission)'
    # Newer Windows can do it in one command. If that fails (older Windows 10),
    # turn on the two Windows features WSL 2 needs instead.
    $cmd = @'
wsl.exe --install --no-distribution
if ($LASTEXITCODE -ne 0) {
    dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
    dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
}
exit $LASTEXITCODE
'@
    $encoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($cmd))
    try {
        $p = Start-Process powershell -ArgumentList "-NoProfile -EncodedCommand $encoded" -Verb RunAs -Wait -PassThru
    } catch {
        Fail 'Installing WSL needs administrator permission. Run setup again and click Yes.'
    }
    # 3010 means it worked and a restart is needed
    if ($p.ExitCode -notin 0, 3010) { Fail "Installing WSL failed (exit code $($p.ExitCode))." }
}

# Run setup again automatically the next time this user logs in
function Register-Resume {
    $runOnce = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\RunOnce'
    # This key doesn't exist on a fresh account until something uses it
    if (-not (Test-Path $runOnce)) { New-Item -Path $runOnce | Out-Null }
    Set-ItemProperty -Path $runOnce -Name 'ROS2DevSetup' -Value "`"$env:ComSpec`" /c `"`"$RepoDir\setup.bat`"`""
}

function Request-Restart {
    Register-Resume
    Write-Host "`nA restart is needed to finish installing. Setup will continue by itself after you log back in." -ForegroundColor Yellow
    $answer = Read-Host 'Restart now? (Y/n)'
    if ($answer -notmatch '^[nN]') { Restart-Computer -Force }
    exit 0
}

function Winget-Install($id, [string[]]$ExtraArgs = @()) {
    winget install -e --id $id --accept-package-agreements --accept-source-agreements @ExtraArgs
    if ($LASTEXITCODE -ne 0) { Fail "Installing $id failed (winget exit code $LASTEXITCODE)." }
}

function Test-Winget { [bool](Get-Command winget -ErrorAction SilentlyContinue) }

function Install-Winget {
    # winget ships with Windows 10/11 as "App Installer" but is sometimes not
    # registered yet for a new user account. Try that first, it's quick.
    Info 'Setting up winget (the Windows package manager)'
    try {
        Add-AppxPackage -RegisterByFamilyName -MainPackage Microsoft.DesktopAppInstaller_8wekyb3d8bbwe
    } catch { }
    Refresh-Path
    if (Test-Winget) { return }

    # Not installed at all: use Microsoft's module to download and install it
    Info 'Installing winget (this can take a few minutes)'
    try {
        [Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12
        $ProgressPreference = 'SilentlyContinue'
        Install-PackageProvider -Name NuGet -Force -Scope CurrentUser | Out-Null
        Install-Module -Name Microsoft.WinGet.Client -Repository PSGallery -Force -Scope CurrentUser | Out-Null
        Import-Module Microsoft.WinGet.Client
        Repair-WinGetPackageManager -Latest -Force
    } catch {
        Fail "Could not install winget: $($_.Exception.Message)`nInstall 'App Installer' from the Microsoft Store, then run this again."
    }
    Refresh-Path
    if (-not (Test-Winget)) { Request-Restart }
}

if (-not (Test-Winget)) { Install-Winget }

# WSL 2 is required by Docker Desktop. WSL and Docker Desktop both need a restart, so install everything first
# and restart once at the end
$NeedsRestart = $false

if (Test-Run { wsl --status }) {
    Write-Host 'WSL is already installed.'
} else {
    Install-Wsl
    $NeedsRestart = $true
}

if (Test-Path $DockerExe) {
    Write-Host "Docker Desktop is already installed ($DockerExe)."
} else {
    Info 'Installing Docker Desktop (click Yes when Windows asks for permission)'
    Write-Host 'The Docker installer window takes 5-10 minutes. Keep the default options.'
    Write-Host 'When it finishes, click its Close button. If it logs you out or restarts,'
    Write-Host 'that is fine: setup continues by itself after you log back in.'
    Register-Resume
    # Show the installer's own window so progress and errors are visible
    Winget-Install 'Docker.DockerDesktop' @('--interactive')
    $NeedsRestart = $true
}

if (Find-Code) {
    Write-Host 'VS Code is already installed.'
} else {
    Info 'Installing VS Code'
    Winget-Install 'Microsoft.VisualStudioCode'
    Refresh-Path
}

$Code = Find-Code
if (-not $Code) { Fail "VS Code was installed but could not be found. Restart your computer and run this again." }

if ((& $Code --list-extensions) -contains $Extension) {
    Write-Host 'The Dev Containers extension is already installed.'
} else {
    Info 'Installing the Dev Containers extension'
    & $Code --install-extension $Extension
}

if ($NeedsRestart) { Request-Restart }

Write-Host 'Checking whether Docker is running...'
if (-not (Test-Docker)) {
    Info 'Starting Docker Desktop (the first time, accept the terms in the Docker window)'
    Start-Process $DockerExe
    Refresh-Path
    Write-Host 'Waiting for Docker to be ready (up to 5 minutes)' -NoNewline
    $deadline = (Get-Date).AddMinutes(5)
    while ((Get-Date) -lt $deadline -and -not (Test-Docker)) {
        Start-Sleep -Seconds 5
        Write-Host '.' -NoNewline
    }
    Write-Host ''
    if (-not (Test-Docker)) {
        Fail ("Docker did not start. Open Docker Desktop from the Start menu to see what is wrong, " +
              "wait until it says 'Engine running', then run this again.`n" +
              "On a virtual machine, Docker needs nested virtualization turned on.")
    }
}

Info 'Opening the project in the dev container (the first build takes several minutes)'
# VS Code identifies a dev container folder by its host path, hex-encoded
$Hex = -join ([Text.Encoding]::UTF8.GetBytes($RepoDir) | ForEach-Object { $_.ToString('x2') })
& $Code --folder-uri "vscode-remote://dev-container+$Hex$ContainerFolder"
