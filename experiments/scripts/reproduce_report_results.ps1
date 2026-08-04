$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..\..")
$srcDir = Join-Path $repoRoot "src"
$resultsDir = Join-Path $repoRoot "experiments\results"

function Run-RepoCommand {
    param(
        [string]$WorkingDirectory,
        [string[]]$Arguments
    )

    Write-Host ""
    Write-Host ">> python $($Arguments -join ' ')" -ForegroundColor Cyan
    Push-Location $WorkingDirectory
    try {
        & python @Arguments
    }
    finally {
        Pop-Location
    }
}

Write-Host "Repository root: $repoRoot" -ForegroundColor Green
Write-Host "Results directory: $resultsDir" -ForegroundColor Green

Run-RepoCommand -WorkingDirectory $repoRoot -Arguments @(
    "experiments/scripts/run_planner_matrix.py",
    "--matrix-name", "planner_matrix_baseline_60s",
    "--time", "1",
    "--agent", "2",
    "--port-load", "1",
    "--port-unload", "1",
    "--size", "12", "12",
    "--resolution", "1",
    "--step", "10"
)

Run-RepoCommand -WorkingDirectory $repoRoot -Arguments @(
    "experiments/scripts/run_planner_matrix.py",
    "--matrix-name", "stage_chain_small_60s",
    "--time", "1",
    "--agent", "2",
    "--port-load", "1",
    "--port-unload", "1",
    "--size", "12", "12",
    "--resolution", "1",
    "--step", "10",
    "--global-planners", "LayeredAStar,LayeredAStarCollisionAware,LayeredAStarReservationAware,LayeredAStarQueueAware",
    "--local-planners", "DullPlanner"
)

Run-RepoCommand -WorkingDirectory $repoRoot -Arguments @(
    "experiments/scripts/run_planner_matrix.py",
    "--matrix-name", "stage_chain_medium_60s",
    "--time", "1",
    "--agent", "4",
    "--port-load", "2",
    "--port-unload", "2",
    "--size", "20", "20",
    "--resolution", "1",
    "--step", "20",
    "--global-planners", "LayeredAStar,LayeredAStarCollisionAware,LayeredAStarReservationAware,LayeredAStarQueueAware",
    "--local-planners", "DullPlanner"
)

Run-RepoCommand -WorkingDirectory $repoRoot -Arguments @(
    "experiments/scripts/run_planner_matrix.py",
    "--matrix-name", "stage_chain_high_60s",
    "--time", "1",
    "--agent", "6",
    "--port-load", "3",
    "--port-unload", "3",
    "--size", "24", "24",
    "--resolution", "1",
    "--step", "20",
    "--global-planners", "LayeredAStar,LayeredAStarCollisionAware,LayeredAStarReservationAware,LayeredAStarQueueAware",
    "--local-planners", "DullPlanner"
)

Run-RepoCommand -WorkingDirectory $srcDir -Arguments @(
    "simulator.py",
    "-t", "1",
    "--default",
    "--agent", "2",
    "--port", "1", "1",
    "--size", "12", "12",
    "--resolution", "1",
    "--step", "10",
    "--gp", "LayeredAStarBaselineTrafficAware",
    "--lp", "DullPlanner",
    "--tm", "NaiveTaskManager",
    "--record-tag", "stage_chain_bta_small_60s_LayeredAStarBaselineTrafficAware_DullPlanner_NaiveTaskManager",
    "--record-dir", $resultsDir
)

Run-RepoCommand -WorkingDirectory $srcDir -Arguments @(
    "simulator.py",
    "-t", "1",
    "--default",
    "--agent", "4",
    "--port", "2", "2",
    "--size", "20", "20",
    "--resolution", "1",
    "--step", "20",
    "--gp", "LayeredAStarBaselineTrafficAware",
    "--lp", "DullPlanner",
    "--tm", "NaiveTaskManager",
    "--record-tag", "stage_chain_bta_medium_60s_LayeredAStarBaselineTrafficAware_DullPlanner_NaiveTaskManager",
    "--record-dir", $resultsDir
)

Run-RepoCommand -WorkingDirectory $srcDir -Arguments @(
    "simulator.py",
    "-t", "1",
    "--default",
    "--agent", "6",
    "--port", "3", "3",
    "--size", "24", "24",
    "--resolution", "1",
    "--step", "20",
    "--gp", "LayeredAStarBaselineTrafficAware",
    "--lp", "DullPlanner",
    "--tm", "NaiveTaskManager",
    "--record-tag", "stage_chain_bta_high_60s_LayeredAStarBaselineTrafficAware_DullPlanner_NaiveTaskManager",
    "--record-dir", $resultsDir
)

Run-RepoCommand -WorkingDirectory $repoRoot -Arguments @(
    "experiments/scripts/generate_final_report_assets.py"
)

Write-Host ""
Write-Host "Reproduction finished." -ForegroundColor Green
Write-Host "Check experiments/results/final_report for regenerated summary tables and plots." -ForegroundColor Green
