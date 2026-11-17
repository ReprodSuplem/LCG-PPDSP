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
    'A-n32-k5', 'A-n33-k5', 'A-n33-k6', 'A-n34-k5', 'A-n36-k5', 'A-n37-k5', 'A-n37-k6', 'A-n38-k5', 'A-n39-k5', 
    'A-n39-k6', 'A-n44-k6', 'A-n45-k6', 'A-n45-k7', 'A-n46-k7', 'A-n48-k7', 'A-n53-k7', 'A-n54-k7', 'A-n55-k9', 
    'A-n60-k9', 'A-n61-k9', 'A-n62-k8', 'A-n63-k10', 'A-n63-k9', 'A-n64-k9', 'A-n65-k9', 'A-n69-k9', 'A-n80-k10', 
    'B-n31-k5', 'B-n34-k5', 'B-n35-k5', 'B-n38-k6', 'B-n39-k5', 'B-n41-k6', 'B-n43-k6', 'B-n44-k7', 
    'B-n45-k5', 'B-n45-k6', 'B-n50-k7', 'B-n50-k8', 'B-n51-k7', 'B-n52-k7', 'B-n56-k7', 'B-n57-k7', 
    'B-n57-k9', 'B-n63-k10', 'B-n64-k9', 'B-n66-k9', 'B-n67-k10', 'B-n68-k9', 'B-n78-k10', 
    'P-n16-k8', 'P-n19-k2', 'P-n20-k2', 'P-n21-k2', 'P-n22-k2', 'P-n22-k8', 'P-n23-k8', 'P-n40-k5', 
    'P-n45-k5', 'P-n50-k7', 'P-n50-k8', 'P-n50-k10', 'P-n51-k10', 'P-n55-k7', 'P-n55-k8', 'P-n55-k10', 
    'P-n55-k15', 'P-n60-k10', 'P-n60-k15', 'P-n65-k10', 'P-n70-k10', 'P-n76-k4', 'P-n76-k5', 'P-n101-k4'
]

# Wall-clock time limit of 3600s per instance, 8 concurrent single-thread workers for a 20-core / 60GB RAM machine
TIME_LIMIT = 3600
MAX_WORKERS = 8
METHODS = ["cpsat", "maxsat_path", "maxsat_order", "static", "hybrid_path", "hybrid_node", "full_path", "full_node"]
K_VALUES = [3, 4, 5]
GLOBAL_SEED = 42

# ==========================================
# Phase 1: Benchmark Instance Synthesis (Section VI-A)
# ==========================================
def generate_data(inst, k):
    vrp_file = f"{inst}.vrp"
    if not os.path.exists(vrp_file):
        print(f"[Warning] Original benchmark file {vrp_file} not found. Assuming CSVs exist.", flush=True)
        return

    print(f"--- Generating Data for {inst} (K={k}, Seed: {GLOBAL_SEED}) ---", flush=True)
    cmd = f"python ppdsp_ins_arg.py {vrp_file} . {GLOBAL_SEED} {k}"
    subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# ==========================================
# Subprocess Execution Wrapper
# ==========================================
def run_task(cmd):
    print(f"Started: {cmd}", flush=True)
    start_time = time.time()
    res = subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    elapsed = time.time() - start_time
    if res.returncode == 0:
        print(f"Finished: {cmd} | Status: Success | Time: {elapsed:.2f}s", flush=True)
        return True
    else:
        err_msg = res.stderr.strip().splitlines()[-1] if res.stderr and res.stderr.strip() else "Unknown error"
        print(f"Finished: {cmd} | Status: Failed | Time: {elapsed:.2f}s | Error: {err_msg}", flush=True)
        return False

# ==========================================
# Phase 2: Batch Solver Execution and Result Aggregation (Section VI-B & VI-C)
# ==========================================
def main():
    inst_to_req = {}
    
    print("========== STEP 1: DATA GENERATION & DISCOVERY ==========", flush=True)
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
                print(f"  [Discovered] {inst} -> Pool size: {req_num}", flush=True)
            except ValueError:
                print(f"  [Error] Could not parse request count from {filename}", flush=True)
        else:
            print(f"  [Warning] No requestInfo file found for {inst} after generation.", flush=True)

    print("\n========== STEP 2: SOLVER TASK QUEUING ==========", flush=True)
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

    print(f"\nTotal tasks queued: {len(tasks)} (Instances x Solvers x K_Values)", flush=True)
    print(f"Starting parallel execution with {MAX_WORKERS} workers...\n", flush=True)

    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as executor:
        outcomes = list(executor.map(run_task, tasks))

    failed_count = outcomes.count(False)
    print(f"\nAll PPDSP experiments completed. Total Failed Tasks: {failed_count}/{len(tasks)}", flush=True)
    print("Triggering results compilation...", flush=True)
    subprocess.run("python ppdsp_results.py", shell=True, check=True)

if __name__ == "__main__":
    main()