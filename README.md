# MILP-FJSP

`MILP-FJSP` is a compact Python implementation of the Flexible Job Shop Scheduling Problem (FJSP) solved by Mixed-Integer Linear Programming (MILP) with Gurobi.

The current repository keeps a single standalone solver script, `MILP-FJSP.py`, and includes bundled Brandimarte benchmark instances under `data/Brandimarte_Data/`.

## Features

- Solves classical FJSP instances with a MILP formulation.
- Uses Gurobi to minimize makespan (`C_max`).
- Prints detailed schedules by job and by machine.
- Computes total energy consumption (TEC) after optimization.
- Supports running a selected list of benchmark instances and exporting a summary CSV.

## Repository Structure

```text
MILP-FJSP/
|-- MILP-FJSP.py
|-- README.md
|-- data/
|   `-- Brandimarte_Data/
|       |-- Mk01.txt
|       |-- Mk02.txt
|       |-- ...
|       `-- Mk10.txt
`-- fjsp_summary.csv
```

## Requirements

- Python 3.10+
- [Gurobi Optimizer](https://www.gurobi.com/)
- `gurobipy`

Install the Python package after Gurobi is installed and licensed:

```bash
pip install gurobipy
```

## Quick Start

Run the solver directly:

```bash
python MILP-FJSP.py
```

The script will:

1. Load the selected dataset instances.
2. Build and solve the MILP model.
3. Print the detailed schedule and energy summary.
4. Save a summary file to `fjsp_summary.csv`.

## Selecting Datasets

Dataset selection is configured in `main()` inside `MILP-FJSP.py`.

Edit the `datasets` list:

```python
datasets = [
    {"name": "Mk01", "da": "Brandimarte_Data"},
    # {"name": "Mk02", "da": "Brandimarte_Data"},
    # {"name": "Mk03", "da": "Brandimarte_Data"},
]
```

Each item contains:

- `name`: instance file name without the `.txt` suffix
- `da`: dataset folder under `data/`

The current repository bundles `Brandimarte_Data`. The code also contains a loader branch for `Bilge and Ulusoy`, but that dataset folder is not included in this repository by default.

## Output

For each solved instance, the script reports:

- number of jobs
- number of machines
- total number of operations
- best makespan
- total energy consumption (TEC)
- solve time
- optimization status

At the end of execution, a CSV summary is written to:

```text
fjsp_summary.csv
```

## Model Summary

The solver builds a MILP model with:

- binary assignment variables for machine selection
- continuous start-time variables for operations
- precedence constraints within each job
- pairwise no-overlap constraints on machines
- makespan minimization objective

Energy consumption is evaluated from the final schedule using:

- processing energy
- idle energy

## Notes

- The primary optimization objective is makespan minimization, not multi-objective optimization.
- TEC is computed after solving and is reported as an evaluation metric.
- Large instances may require substantial time depending on the Gurobi license and hardware.
