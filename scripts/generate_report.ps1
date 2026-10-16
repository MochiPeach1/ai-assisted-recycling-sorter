$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    & docker compose up -d
    if ($LASTEXITCODE -ne 0) {
        throw "Docker Compose could not start the report-generation environment."
    }

    & docker compose exec -T sim python3 /workspace/project/scripts/generate_report.py
    if ($LASTEXITCODE -ne 0) {
        throw "The report PDF generator failed inside the course container."
    }
}
finally {
    Pop-Location
}
