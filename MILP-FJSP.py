"""
Solve the Flexible Job Shop Scheduling Problem (FJSP) with Gurobi.
"""

import csv
import time
from pathlib import Path

import gurobipy as gp
from gurobipy import GRB


class FJSPSolver:
    """FJSP solver."""

    PROCESSING_POWER = 30.0
    IDLE_POWER = 1.0
    GUROBI_TIME_LIMIT = 3600
    BIG_M = 1000

    def __init__(self):
        self.jobs = []
        self.machines = []
        self.processing_times = {}
        self.machine_capability = {}
        self.job_operations = {}
        self.num_jobs = 0
        self.num_machines = 0
        self.avg_operations = 0

    def _parse_data_file(self, filepath):
        """Parse a standard FJSP instance file."""
        with open(filepath, "r", encoding="utf-8") as file:
            lines = file.readlines()

        first_line = lines[0].strip().split()
        self.num_jobs = int(first_line[0])
        self.num_machines = int(first_line[1])
        self.avg_operations = float(first_line[2])

        self.jobs = list(range(self.num_jobs))
        self.machines = list(range(self.num_machines))
        self.job_operations = {}
        self.machine_capability = {}
        self.processing_times = {}

        for job_idx in range(self.num_jobs):
            line = [int(x) for x in lines[job_idx + 1].strip().split()]
            num_operations = line[0]
            self.job_operations[job_idx] = list(range(num_operations))

            pos = 1
            for op_idx in range(num_operations):
                num_machines_for_op = line[pos]
                pos += 1

                available_machines = []
                for _ in range(num_machines_for_op):
                    machine_id = line[pos] - 1
                    processing_time = line[pos + 1]

                    available_machines.append(machine_id)
                    self.processing_times[(job_idx, op_idx, machine_id)] = processing_time
                    pos += 2

                self.machine_capability[(job_idx, op_idx)] = available_machines

    def load_mk_data(self, filename):
        """Load Brandimarte MK data."""
        try:
            print(f"Loading MK dataset: {filename}")
            self._parse_data_file(filename)
            print(
                f"Jobs: {self.num_jobs}, Machines: {self.num_machines}, "
                f"Avg operations: {self.avg_operations}"
            )
            print(
                "MK dataset loaded. "
                f"Total operations: {sum(len(ops) for ops in self.job_operations.values())}"
            )
            return True
        except Exception as exc:
            print(f"Failed to load MK dataset: {exc}")
            return False

    def load_bilge_ulusoy_data(self, job_filename):
        """Load Bilge and Ulusoy data."""
        try:
            print(f"Loading Bilge and Ulusoy dataset: {job_filename}")
            self._parse_data_file(job_filename)
            print(
                f"Jobs: {self.num_jobs}, Machines: {self.num_machines}, "
                f"Avg operations: {self.avg_operations}"
            )
            print(
                "Bilge and Ulusoy dataset loaded. "
                f"Total operations: {sum(len(ops) for ops in self.job_operations.values())}"
            )
            return True
        except Exception as exc:
            print(f"Failed to load Bilge and Ulusoy dataset: {exc}")
            return False

    def calculate_tec(self, schedule, makespan):
        """Calculate total energy consumption."""
        machine_processing_time = {machine: 0 for machine in self.machines}

        for item in schedule:
            if item["machine"] is not None:
                machine_processing_time[item["machine"]] += item["processing_time"]

        total_processing_time = sum(machine_processing_time.values())
        total_idle_time = sum(makespan - machine_processing_time[m] for m in self.machines)

        processing_energy = total_processing_time * self.PROCESSING_POWER
        idle_energy = total_idle_time * self.IDLE_POWER
        total_energy = processing_energy + idle_energy

        return {
            "total_energy": total_energy,
            "processing_energy": processing_energy,
            "idle_energy": idle_energy,
            "total_processing_time": total_processing_time,
            "total_idle_time": total_idle_time,
            "machine_processing_time": machine_processing_time,
        }

    def _build_model(self):
        """Build the Gurobi model."""
        model = gp.Model("FJSP")
        model.setParam("TimeLimit", self.GUROBI_TIME_LIMIT)

        x = {}
        for job in self.jobs:
            for op in self.job_operations[job]:
                for machine in self.machine_capability.get((job, op), []):
                    x[job, op, machine] = model.addVar(
                        vtype=GRB.BINARY,
                        name=f"x_{job}_{op}_{machine}",
                    )

        s = {}
        for job in self.jobs:
            for op in self.job_operations[job]:
                s[job, op] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name=f"s_{job}_{op}",
                )

        c_max = model.addVar(vtype=GRB.CONTINUOUS, lb=0, name="C_max")

        model.setObjective(c_max, GRB.MINIMIZE)

        for job in self.jobs:
            for op in self.job_operations[job]:
                model.addConstr(
                    gp.quicksum(
                        x[job, op, machine]
                        for machine in self.machine_capability.get((job, op), [])
                    )
                    == 1,
                    f"assign_{job}_{op}",
                )

        for job in self.jobs:
            operations = self.job_operations[job]
            for i in range(len(operations) - 1):
                current_op = operations[i]
                next_op = operations[i + 1]

                processing_time = gp.quicksum(
                    self.processing_times.get((job, current_op, machine), 0)
                    * x[job, current_op, machine]
                    for machine in self.machine_capability.get((job, current_op), [])
                )

                model.addConstr(
                    s[job, current_op] + processing_time <= s[job, next_op],
                    f"precedence_{job}_{current_op}_{next_op}",
                )

        for machine in self.machines:
            operations_on_machine = []
            for job in self.jobs:
                for op in self.job_operations[job]:
                    if machine in self.machine_capability.get((job, op), []):
                        operations_on_machine.append((job, op))

            for i in range(len(operations_on_machine)):
                for j in range(i + 1, len(operations_on_machine)):
                    job1, op1 = operations_on_machine[i]
                    job2, op2 = operations_on_machine[j]

                    y = model.addVar(
                        vtype=GRB.BINARY,
                        name=f"y_{job1}_{op1}_{job2}_{op2}_{machine}",
                    )

                    p1 = self.processing_times.get((job1, op1, machine), 0)
                    p2 = self.processing_times.get((job2, op2, machine), 0)

                    model.addConstr(
                        s[job1, op1] + p1
                        <= s[job2, op2]
                        + self.BIG_M * (1 - y)
                        + self.BIG_M * (2 - x[job1, op1, machine] - x[job2, op2, machine]),
                        f"no_overlap_1_{job1}_{op1}_{job2}_{op2}_{machine}",
                    )

                    model.addConstr(
                        s[job2, op2] + p2
                        <= s[job1, op1]
                        + self.BIG_M * y
                        + self.BIG_M * (2 - x[job1, op1, machine] - x[job2, op2, machine]),
                        f"no_overlap_2_{job1}_{op1}_{job2}_{op2}_{machine}",
                    )

        for job in self.jobs:
            last_op = self.job_operations[job][-1]
            processing_time = gp.quicksum(
                self.processing_times.get((job, last_op, machine), 0) * x[job, last_op, machine]
                for machine in self.machine_capability.get((job, last_op), [])
            )
            model.addConstr(
                s[job, last_op] + processing_time <= c_max,
                f"completion_{job}",
            )

        return model, x, s, c_max

    def _extract_schedule(self, x, s):
        """Extract the schedule from the optimized model."""
        schedule = []
        for job in self.jobs:
            for op in self.job_operations[job]:
                start_time = s[job, op].x
                selected_machine = None
                processing_time = 0

                for machine in self.machine_capability.get((job, op), []):
                    if x[job, op, machine].x > 0.5:
                        selected_machine = machine
                        processing_time = self.processing_times.get((job, op, machine), 0)
                        break

                end_time = start_time + processing_time
                schedule.append(
                    {
                        "job": job,
                        "operation": op,
                        "machine": selected_machine,
                        "start_time": start_time,
                        "end_time": end_time,
                        "processing_time": processing_time,
                    }
                )

        return schedule

    def _print_schedule(self, schedule):
        """Print the detailed schedule."""
        print("\nDetailed schedule:")
        for job in self.jobs:
            print(f"\nJob {job}:")
            for item in schedule:
                if item["job"] == job:
                    print(
                        f"  Op {item['operation']}: Machine {item['machine']}, "
                        f"Time [{item['start_time']:.2f}, {item['end_time']:.2f}], "
                        f"Process {item['processing_time']}"
                    )

        print("\nSchedule grouped by machine:")
        for machine in self.machines:
            machine_schedule = [item for item in schedule if item["machine"] == machine]
            machine_schedule.sort(key=lambda item: item["start_time"])

            if machine_schedule:
                print(f"\nMachine {machine}:")
                for item in machine_schedule:
                    print(
                        f"  Job{item['job']}-Op{item['operation']}: "
                        f"[{item['start_time']:.2f}, {item['end_time']:.2f}]"
                    )

    def solve_fjsp(self):
        """Solve the FJSP instance."""
        try:
            print("Start solving FJSP...")

            model, x, s, c_max = self._build_model()
            model.optimize()

            if model.status == GRB.OPTIMAL:
                print("\n" + "=" * 60)
                print("FJSP solved successfully.")
                print(f"Best makespan: {c_max.x:.2f}")
                print("=" * 60)

                schedule = self._extract_schedule(x, s)
                self._print_schedule(schedule)

                tec = self.calculate_tec(schedule, c_max.x)
                print("\nEnergy summary:")
                print(f"Total energy (TEC): {tec['total_energy']:.2f} kWh")
                print(f"  Processing energy: {tec['processing_energy']:.2f} kWh")
                print(f"  Idle energy: {tec['idle_energy']:.2f} kWh")
                print(f"  Total processing time: {tec['total_processing_time']:.2f}")
                print(f"  Total idle time: {tec['total_idle_time']:.2f}")

                return c_max.x, schedule, tec

            if model.status == GRB.TIME_LIMIT:
                print("Time limit reached, returning the best incumbent.")
                if model.solCount > 0:
                    print(f"Current best solution: {c_max.x:.2f}")
                    schedule = self._extract_schedule(x, s)
                    tec = self.calculate_tec(schedule, c_max.x)
                    return c_max.x, schedule, tec

                print("No feasible solution found.")
                return None, None, None

            print(f"Optimization failed with status code: {model.status}")
            return None, None, None

        except gp.GurobiError as exc:
            print(f"Gurobi error: {exc}")
            return None, None, None
        except Exception as exc:
            print(f"Other error: {exc}")
            return None, None, None


def _ensure_txt(name):
    if name.lower().endswith(".txt"):
        return name
    return f"{name}.txt"


def run_selected_datasets(datasets, base_dir="data", output_csv="fjsp_summary.csv"):
    """Run the solver on an explicit dataset selection list."""
    summary = []

    for item in datasets:
        name = item["name"]
        data_folder = item["da"]

        dataset_dir = Path(base_dir) / data_folder
        job_path = dataset_dir / _ensure_txt(name)

        print(f"\n=== Data: {data_folder} | Jobset: {job_path.name} ===")

        if not dataset_dir.exists():
            print(f"Error: dataset directory not found: {dataset_dir}")
            summary.append(
                {
                    "dataset": data_folder,
                    "job": job_path.name,
                    "status": "missing_dir",
                    "jobs": None,
                    "machines": None,
                    "operations": None,
                    "makespan": None,
                    "tec": None,
                    "time": 0.0,
                }
            )
            continue

        if not job_path.exists():
            print(f"Warning: file not found, skipped: {job_path.name}")
            summary.append(
                {
                    "dataset": data_folder,
                    "job": job_path.name,
                    "status": "missing_file",
                    "jobs": None,
                    "machines": None,
                    "operations": None,
                    "makespan": None,
                    "tec": None,
                    "time": 0.0,
                }
            )
            continue

        solver = FJSPSolver()
        if data_folder == "Brandimarte_Data":
            loaded = solver.load_mk_data(str(job_path))
        elif data_folder == "Bilge and Ulusoy":
            loaded = solver.load_bilge_ulusoy_data(str(job_path))
        else:
            print(f"Error: unsupported dataset folder: {data_folder}")
            summary.append(
                {
                    "dataset": data_folder,
                    "job": job_path.name,
                    "status": "unsupported_dataset",
                    "jobs": None,
                    "machines": None,
                    "operations": None,
                    "makespan": None,
                    "tec": None,
                    "time": 0.0,
                }
            )
            continue

        if not loaded:
            summary.append(
                {
                    "dataset": data_folder,
                    "job": job_path.name,
                    "status": "load_failed",
                    "jobs": None,
                    "machines": None,
                    "operations": None,
                    "makespan": None,
                    "tec": None,
                    "time": 0.0,
                }
            )
            continue

        operations = sum(len(ops) for ops in solver.job_operations.values())
        print("\nProblem scale:")
        print(f"Jobs: {solver.num_jobs}")
        print(f"Machines: {solver.num_machines}")
        print(f"Average operations: {solver.avg_operations}")
        print(f"Total operations: {operations}\n")

        start_time = time.time()
        makespan, schedule, tec = solver.solve_fjsp()
        solve_time = time.time() - start_time

        result = {
            "dataset": data_folder,
            "job": job_path.name,
            "jobs": solver.num_jobs,
            "machines": solver.num_machines,
            "operations": operations,
            "makespan": makespan,
            "tec": tec["total_energy"] if tec else None,
            "time": solve_time,
            "status": "Optimal" if makespan is not None else "Failed",
        }
        summary.append(result)

        if makespan is not None:
            print(f"\nSolved: {job_path.stem}")
            print(f"Best makespan: {makespan:.2f}")
            if tec:
                print(f"TEC: {tec['total_energy']:.2f} kWh")
            print(f"Solve time: {solve_time:.2f} s\n")
        else:
            print(f"\nFailed: {job_path.stem}")
            print(f"Solve time: {solve_time:.2f} s\n")

    print("\n==== Summary ====")
    header = (
        f"{'Dataset':<20} {'Jobset':<12} {'Jobs':<6} {'Machines':<9} "
        f"{'Ops':<6} {'Makespan':<10} {'TEC':<10} {'Time(s)':<10} {'Status':<18}"
    )
    print(header)
    print("-" * len(header))
    for item in summary:
        jobs_str = str(item["jobs"]) if item["jobs"] is not None else "-"
        machines_str = str(item["machines"]) if item["machines"] is not None else "-"
        operations_str = str(item["operations"]) if item["operations"] is not None else "-"
        makespan_str = f"{item['makespan']:.2f}" if item["makespan"] is not None else "-"
        tec_str = f"{item['tec']:.2f}" if item["tec"] is not None else "-"
        time_str = f"{item['time']:.2f}"
        print(
            f"{item['dataset']:<20} {item['job']:<12} {jobs_str:<6} {machines_str:<9} "
            f"{operations_str:<6} {makespan_str:<10} {tec_str:<10} {time_str:<10} "
            f"{item['status']:<18}"
        )

    successful = [item for item in summary if item["makespan"] is not None]
    print("\nStatistics:")
    print(f"Total instances: {len(summary)}")
    print(f"Solved instances: {len(successful)}")
    if summary:
        print(f"Success rate: {len(successful) / len(summary) * 100:.1f}%")

    if successful:
        avg_time = sum(item["time"] for item in successful) / len(successful)
        avg_makespan = sum(item["makespan"] for item in successful) / len(successful)
        tec_items = [item["tec"] for item in successful if item["tec"] is not None]
        print(f"Average solve time: {avg_time:.2f} s")
        print(f"Average makespan: {avg_makespan:.2f}")
        if tec_items:
            avg_tec = sum(tec_items) / len(tec_items)
            print(f"Average TEC: {avg_tec:.2f} kWh")

    if output_csv:
        with open(output_csv, "w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "dataset",
                    "jobset",
                    "jobs",
                    "machines",
                    "operations",
                    "status",
                    "makespan",
                    "tec",
                    "time_sec",
                ]
            )
            for item in summary:
                writer.writerow(
                    [
                        item["dataset"],
                        item["job"],
                        item["jobs"] if item["jobs"] is not None else "",
                        item["machines"] if item["machines"] is not None else "",
                        item["operations"] if item["operations"] is not None else "",
                        item["status"],
                        f"{item['makespan']:.4f}" if item["makespan"] is not None else "",
                        f"{item['tec']:.4f}" if item["tec"] is not None else "",
                        f"{item['time']:.4f}",
                    ]
                )
        print(f"\nCSV written: {output_csv}")

    return summary


def main():
    print("FJSP solver - multi-dataset test")
    print("=" * 60 + "\n")

    datasets = [
        {"name": "Mk01", "da": "Brandimarte_Data"},
        # {"name": "Mk02", "da": "Brandimarte_Data"},
        # {"name": "Mk03", "da": "Brandimarte_Data"},
        # {"name": "Mk04", "da": "Brandimarte_Data"},
        # {"name": "Mk05", "da": "Brandimarte_Data"},
        # {"name": "Mk06", "da": "Brandimarte_Data"},
        # {"name": "Mk07", "da": "Brandimarte_Data"},
        # {"name": "Mk08", "da": "Brandimarte_Data"},
        # {"name": "Mk09", "da": "Brandimarte_Data"},
        # {"name": "Mk10", "da": "Brandimarte_Data"},
        
    ]

    results = run_selected_datasets(datasets, base_dir="data")
    print(f"\nCompleted. Processed {len(results)} dataset items.")


if __name__ == "__main__":
    main()
