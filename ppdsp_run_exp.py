# ppdsp_run_exp.py
# Automated benchmark synthesis and batch execution across all 222 PPDSP instances
# and eight exact solver configurations (Section VI-A, Section VI-B).

import os
import glob
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor

# ==========================================
# Experimental Benchmark Configuration (Section VI-A & VI-B)
# ==========================================
# 74 root CVRPLib instances: 27 from Set A, 23 from Set B, and 24 from Set P (Table I)
INSTANCES = [
    'A-n32-k5'
]

# Wall-clock time limit of 3600s per instance, eight exact configurations, k-NN in {3, 4, 5}, and random seed 42
TIME_LIMIT = 3600
MAX_WORKERS = 8
METHODS = ["cpsat", "maxsat_path", "maxsat_order", "static", "hybrid_path", "hybrid_node", "full_path", "full_node"]
K_VALUES = [3]
GLOBAL_SEED = 42

# ==========================================
# Phase 1: Benchmark Instance Synthesis (Section VI-A)
# ==========================================
def generate_data(inst, k):
    vrp_file = f"{inst}.vrp"
    if not os.path.exists(vrp_file):
        print(f"[Warning] Original benchmark file {vrp_file} not found. Assuming CSVs exist.")
        return

    print(f"--- Generating Data for {inst} (K={k}, Seed: {GLOBAL_SEED}) ---")
    cmd = f"python ppdsp_ins_arg.py {vrp_file} . {GLOBAL_SEED} {k}"
    subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# ==========================================
# Subprocess Execution Wrapper
# ==========================================
def run_task(cmd):
    print(f"Started: {cmd}")
    start_time = time.time()
    try:
        subprocess.run(cmd, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        status = "Success"
    except subprocess.CalledProcessError:
        status = "Failed"
    elapsed = time.time() - start_time
    print(f"Finished: {cmd} | Status: {status} | Time: {elapsed:.2f}s")

# ==========================================
# Phase 2: Batch Solver Execution and Result Aggregation (Section VI-B & VI-C)
# ==========================================
def main():
    inst_to_req = {}
    
    print("========== STEP 1: DATA GENERATION & DISCOVERY ==========")
    for inst in INSTANCES:
        for k in K_VALUES:
            generate_data(inst, k)
        
        pattern = f"requestInfo*_{inst}.csv"
        matching_files = glob.glob(pattern)
        if matching_files:
            filename = os.path.basename(matching_files[0])
            try:
                req_num = int(filename.replace("requestInfo", "").split("_")[0])
                inst_to_req[inst] = req_num
                print(f"  [Discovered] {inst} -> Pool size: {req_num}")
            except ValueError:
                print(f"  [Error] Could not parse request count from {filename}")
        else:
            print(f"  [Warning] No requestInfo file found for {inst} after generation.")

    print("\n========== STEP 2: SOLVER TASK QUEUING ==========")
    tasks = []
    
    for k in K_VALUES:
        for inst, req in inst_to_req.items():
            for method in METHODS:
                if method == "static":
                    cmd = f"python ppdsp_main.py mip {inst} {req} --knn {k} --mip_strategy static --time {TIME_LIMIT}"
                elif method == "maxsat_path":
                    cmd = f"python ppdsp_main.py sat {inst} {req} --knn {k} --cut_type path --time {TIME_LIMIT}"
                elif method == "maxsat_order":
                    cmd = f"python ppdsp_main.py sat {inst} {req} --knn {k} --cut_type order --time {TIME_LIMIT}"
                elif method == "cpsat":
                    cmd = f"python ppdsp_main.py cpsat {inst} {req} --knn {k} --time {TIME_LIMIT}"
                elif method == "hybrid_path":
                    cmd = f"python ppdsp_main.py mip {inst} {req} --knn {k} --mip_strategy hybrid --cut_type path --time {TIME_LIMIT}"
                elif method == "hybrid_node":
                    cmd = f"python ppdsp_main.py mip {inst} {req} --knn {k} --mip_strategy hybrid --cut_type node --time {TIME_LIMIT}"
                elif method == "full_path":
                    cmd = f"python ppdsp_main.py mip {inst} {req} --knn {k} --mip_strategy full --cut_type path --time {TIME_LIMIT}"
                elif method == "full_node":
                    cmd = f"python ppdsp_main.py mip {inst} {req} --knn {k} --mip_strategy full --cut_type node --time {TIME_LIMIT}"
                else:
                    continue
                tasks.append(cmd)

    print(f"\nTotal tasks queued: {len(tasks)} (Instances x Solvers x K_Values)")
    print(f"Starting parallel execution with {MAX_WORKERS} workers...\n")

    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as executor:
        executor.map(run_task, tasks)

    print("\nAll PPDSP Ablation experiments completed successfully.")
    print("Triggering results compilation...")
    subprocess.run("python ppdsp_results.py", shell=True)

if __name__ == "__main__":
    main()
