#!/usr/bin/env pwsh
#
# Windows wrapper around the Docker-only build (mirrors ./build.sh).
#
# Every compilation still happens inside the per-architecture images; Docker
# Desktop for Windows is required and no compiler is needed on the host.
# Unlike build.sh, the repository is mounted at /work inside the container:
# Windows paths (C:\...) cannot be reused as Linux paths, and the program only
# uses paths relative to its working directory, so the mount point is free.

$ErrorActionPreference = 'Stop'

$RepoRoot = $PSScriptRoot
$ImageX86 = 'cdfr-builder-x86_64'
$ImageArm = 'cdfr-builder-arm64'
$ContainerWork = '/work'

function Write-Log([string]$Message) {
    Write-Host '==> ' -ForegroundColor Blue -NoNewline
    Write-Host $Message
}

function Assert-Docker {
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        throw 'docker was not found in PATH. Install Docker Desktop for Windows and reopen the terminal.'
    }
    docker info *> $null
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
    docker image inspect $Image *> $null
    if ($LASTEXITCODE -ne 0) { Build-Image $Image $Dockerfile }
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

function Build-Arch([string]$Arch) {
    $spec = Get-ArchSpec $Arch
    Ensure-Image $spec.Image $spec.Dockerfile
    Write-Log "Building $Arch"
    $configureArgs = @('cmake', '--preset', $Arch)
    if ($Arch -eq 'x86_64') {
        # The Linux build symlinks compile_commands.json into the source tree;
        # creating a symlink on the Windows bind mount is unreliable, so it is
        # disabled here (the ARM preset does not export it while cross-compiling).
        $configureArgs += '-DCMAKE_EXPORT_COMPILE_COMMANDS=OFF'
    }
    Invoke-Container -Image $spec.Image -CommandArgs $configureArgs
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
    Write-Log "Running programCDFR from build/x86_64 (host port $port -> container 80)"
    Invoke-Container -Image $ImageX86 `
        -DockerArgs @('-p', "${port}:80") `
        -WorkDir "${ContainerWork}/build/x86_64" `
        -CommandArgs (@('./programCDFR') + $ProgramArgs)
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
