#!/usr/bin/env pwsh
#
# Windows wrapper around the Docker-only build (mirrors ./build.sh). Requires
# Docker Desktop; no compiler or CMake is needed on the host. Unlike build.sh, the
# repo is mounted at /work (Windows paths cannot be reused as Linux paths, and the
# program only uses relative paths, so the mount point is free).

$ErrorActionPreference = 'Stop'

$RepoRoot = $PSScriptRoot
$ImageX86 = 'cdfr-builder-x86_64'
$ImageArm = 'cdfr-builder-arm64'
$ContainerWork = '/work'

function Write-Log([string]$Message) {
    Write-Host '==> ' -ForegroundColor Blue -NoNewline
    Write-Host $Message
}

function Ensure-Submodules {
    $submoduleSrc = Join-Path $RepoRoot 'dependencies\rplidar_sdk\sdk\src\sl_lidar_driver.cpp'
    if (Test-Path $submoduleSrc) { return }

    Write-Log 'Initializing git submodules'
    git -C $RepoRoot submodule sync --recursive
    if ($LASTEXITCODE -ne 0) { throw 'git submodule sync failed' }
    git -C $RepoRoot submodule update --init --recursive
    if ($LASTEXITCODE -ne 0) { throw 'git submodule update failed' }

    if (-not (Test-Path $submoduleSrc)) {
        throw 'The RPLIDAR SDK submodule is still missing. Check git submodule status and ensure dependencies/rplidar_sdk is populated.'
    }
}

function Assert-Docker {
    Ensure-Submodules
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        throw 'docker was not found in PATH. Install Docker Desktop for Windows and reopen the terminal.'
    }
    try {
        & docker info --format '{{.ServerVersion}}' *> $null
    } catch {
        throw 'The Docker daemon is not reachable. Start Docker Desktop and try again.'
    }
    if ($LASTEXITCODE -ne 0) {
        throw 'The Docker daemon is not reachable. Start Docker Desktop and try again.'
    }
}

function Build-Image([string]$Image, [string]$Dockerfile) {
    Write-Log "Building image $Image"
    docker build -t $Image -f $Dockerfile .
    if ($LASTEXITCODE -ne 0) { throw "Failed to build image $Image" }
}

function Ensure-Image([string]$Image, [string]$Dockerfile) {
    $exists = $false
    try {
        & docker image inspect $Image *> $null
        $exists = ($LASTEXITCODE -eq 0)
    } catch {
        $exists = $false
    }
    if (-not $exists) { Build-Image $Image $Dockerfile }
}

function Get-ArchSpec([string]$Arch) {
    switch ($Arch) {
        'x86_64' { return @{ Image = $ImageX86; Dockerfile = 'docker/Dockerfile.x86_64' } }
        'arm64'  { return @{ Image = $ImageArm; Dockerfile = 'docker/Dockerfile.arm64' } }
        default  { throw "Unknown architecture: $Arch (expected x86_64 or arm64)" }
    }
}

# Invoke-Container <image> [-DockerArgs ...] [-WorkDir ...] [-CommandArgs ...]
function Invoke-Container {
    param(
        [Parameter(Mandatory)][string]$Image,
        [string[]]$DockerArgs = @(),
        [string]$WorkDir = $ContainerWork,
        [string[]]$CommandArgs = @()
    )
    $runArgs = @('run', '--rm')
    if (-not [Console]::IsOutputRedirected) { $runArgs += '-it' }
    $runArgs += $DockerArgs
    $runArgs += @('-v', "${RepoRoot}:${ContainerWork}", '-w', $WorkDir, $Image)
    $runArgs += $CommandArgs
    docker @runArgs
    if ($LASTEXITCODE -ne 0) { throw "docker run failed with exit code $LASTEXITCODE" }
}

function Configure-Arch([string]$Arch) {
    $spec = Get-ArchSpec $Arch
    $buildDir = Join-Path $RepoRoot "build\$Arch"
    $cacheFile = Join-Path $buildDir 'CMakeCache.txt'
    $needsConfigure = -not (Test-Path $cacheFile)

    if ($needsConfigure) {
        Write-Log "Configuring $Arch"
        $configureArgs = @('cmake', '--preset', $Arch)
        if ($Arch -eq 'x86_64') {
            $configureArgs += '-DCMAKE_EXPORT_COMPILE_COMMANDS=OFF'
        }
        Invoke-Container -Image $spec.Image -CommandArgs $configureArgs
    }
}

function Build-Arch([string]$Arch) {
    $spec = Get-ArchSpec $Arch
    Ensure-Image $spec.Image $spec.Dockerfile
    Configure-Arch $Arch
    Write-Log "Building $Arch"
    Invoke-Container -Image $spec.Image -CommandArgs @('cmake', '--build', '--preset', $Arch)
}

function Deploy {
    $spec = Get-ArchSpec 'arm64'
    Ensure-Image $spec.Image $spec.Dockerfile
    # The container runs as root, so ssh reads /root/.ssh; mounting the Windows
    # key directory there authenticates the rsync/ssh deploy without an agent.
    $dockerArgs = @()
    $sshDir = Join-Path $HOME '.ssh'
    if (Test-Path $sshDir) { $dockerArgs += @('-v', "${sshDir}:/root/.ssh:ro") }
    Write-Log 'Deploying to robot'
    Invoke-Container -Image $spec.Image -DockerArgs $dockerArgs -CommandArgs @('cmake', '--preset', 'arm64')
    Invoke-Container -Image $spec.Image -DockerArgs $dockerArgs -CommandArgs @('cmake', '--build', '--preset', 'arm64', '--target', 'deploy')
}

function Run-Program([string[]]$ProgramArgs) {
    Build-Arch 'x86_64'
    $port = if ($env:CDFR_RUN_PORT) { $env:CDFR_RUN_PORT } else { '80' }
    $containerArgs = @('sudo', './programCDFR') + $ProgramArgs
    Write-Log "Running programCDFR from build/x86_64 (host port $port -> container 80)"
    Invoke-Container -Image $ImageX86 `
        -DockerArgs @('-p', "${port}:80", '--cap-add=SYS_NICE') `
        -WorkDir "${ContainerWork}/build/x86_64" `
        -CommandArgs $containerArgs
}

function Open-Shell([string]$Arch) {
    $spec = Get-ArchSpec $Arch
    Ensure-Image $spec.Image $spec.Dockerfile
    Invoke-Container -Image $spec.Image -CommandArgs @('bash')
}

function Show-Usage {
    @'
Usage: build.bat <command> [arch|args...]

Commands:
  build [x86_64|arm64]   Build the given target, or both when omitted
  run [args...]          Build x86_64 and run the program locally (REST API on port 80)
  test                   Build x86_64 and run the CTest suite
  deploy                 Build arm64 and deploy it to the robot
  shell [x86_64|arm64]   Interactive shell in the target image (default: x86_64)
  images                 (Re)build both Docker images
  clean                  Remove the build/ directory

Environment:
  CDFR_RUN_PORT          Host port published for ./programCDFR (default: 80)
'@
}

$hasCommand = $args.Count -ge 1
$command = if ($hasCommand) { $args[0] } else { 'build' }
$rest = if ($args.Count -ge 2) { $args[1..($args.Count - 1)] } else { @() }

Push-Location $RepoRoot
try {
    switch ($command) {
        'build' {
            Assert-Docker
            if ($rest.Count -ge 1) { Build-Arch $rest[0] }
            else { Build-Arch 'x86_64'; Build-Arch 'arm64' }
        }
        'test' {
            Assert-Docker
            Build-Arch 'x86_64'
            Write-Log 'Running tests'
            Invoke-Container -Image $ImageX86 -CommandArgs @('ctest', '--preset', 'x86_64')
        }
        'run' {
            Assert-Docker
            Run-Program $rest
        }
        'deploy' {
            Assert-Docker
            Deploy
        }
        'shell' {
            Assert-Docker
            Open-Shell $(if ($rest.Count -ge 1) { $rest[0] } else { 'x86_64' })
        }
        'images' {
            Assert-Docker
            Build-Image $ImageX86 'docker/Dockerfile.x86_64'
            Build-Image $ImageArm 'docker/Dockerfile.arm64'
        }
        'clean' {
            Remove-Item -Recurse -Force -ErrorAction SilentlyContinue (Join-Path $RepoRoot 'build')
            Remove-Item -Force -ErrorAction SilentlyContinue (Join-Path $RepoRoot 'compile_commands.json')
            Write-Log 'Build artifacts removed'
        }
        { $_ -in '-h', '--help', 'help' } {
            Show-Usage
        }
        default {
            Show-Usage
            exit 1
        }
    }
}
finally {
    Pop-Location
}
