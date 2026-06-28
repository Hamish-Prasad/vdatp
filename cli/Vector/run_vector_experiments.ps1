param(
    [int]$Iterations = 450,
    [int]$OptimizeTimeoutSeconds = 180,
    [int]$VerifyTimeoutSeconds = 240,
    [int]$Frames = 4,
    [switch]$RunStaticResearch
)

$ErrorActionPreference = "Continue"
$python = "C:\Users\Bored\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
$env:PYTHONDONTWRITEBYTECODE = "1"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$results = Join-Path $here "results"
$log = Join-Path $results "run.log"
New-Item -ItemType Directory -Force -Path $results | Out-Null

"[$(Get-Date -Format o)] run started" | Tee-Object -FilePath $log
if ($RunStaticResearch) {
    "[$(Get-Date -Format o)] static research optimization started" | Tee-Object -FilePath $log -Append
    & $python (Join-Path $here "vector_optimizer.py") --case all --iterations $Iterations --frames $Frames --max-seconds $OptimizeTimeoutSeconds --out-dir $results 2>&1 | Tee-Object -FilePath $log -Append
    if ($LASTEXITCODE -ne 0) { throw "optimizer exited with code $LASTEXITCODE" }
    & $python (Join-Path $here "verify_and_visualize.py") --case all --results $results --screenshot --max-seconds $VerifyTimeoutSeconds 2>&1 | Tee-Object -FilePath $log -Append
    if ($LASTEXITCODE -ne 0) { throw "verifier exited with code $LASTEXITCODE" }
}
"[$(Get-Date -Format o)] dynamic vector policy started" | Tee-Object -FilePath $log -Append
& $python (Join-Path $here "dynamic_vector_policy.py") --case all --out-dir $results --screenshot --max-seconds $VerifyTimeoutSeconds 2>&1 | Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) { throw "dynamic policy verifier exited with code $LASTEXITCODE" }
"[$(Get-Date -Format o)] complete" | Tee-Object -FilePath $log -Append
