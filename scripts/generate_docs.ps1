# PowerShell script to generate documentation using Doxygen
# Usage: .\generate_docs.ps1

# Check for doxygen command
if (-not (Get-Command doxygen -ErrorAction SilentlyContinue)) {
    Write-Error "Doxygen executable not found in PATH. Please install Doxygen (https://www.doxygen.nl/) and ensure it is available in your PATH."
    exit 1
}

```powershell
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Push-Location $root
```

Write-Host "Running Doxygen with Doxyfile at: $root\Doxyfile"
& doxygen Doxyfile
if ($LASTEXITCODE -ne 0) {
    Write-Error "Doxygen failed with exit code $LASTEXITCODE"
    Pop-Location
    exit $LASTEXITCODE
}

$index = Join-Path -Path $root -ChildPath "docs/doxygen/html/index.html"
if (Test-Path $index) {
    Write-Host "Documentation generated at: $index"
    Start-Process $index
}
else {
    Write-Host "Documentation generated but index.html not found. Check docs/doxygen/html folder."
}
Pop-Location
