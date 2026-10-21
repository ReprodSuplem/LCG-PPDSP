# ppdsp_results.py
# Log parser and evaluation metrics aggregator for primal objectives, Best Known Bounds (BKB),
# relative optimality gaps, and convergence runtimes (Section VI-C).

import os
import re
import pandas as pd
import numpy as np

def parse_log_file(filepath):
    """
    Parse an individual solver log (.out) to extract primal objective value z, validated
    dual upper bound, time to best incumbent T_best, total execution time T_total, and
    termination status (Section VI-C).
    """
    filename = os.path.basename(filepath)
    match = re.match(r"ppdsp_(.*)_r(\d+)v(\d+)k(\d+)_(.*)\.out", filename)
    if not match:
        return None

    instance = match.group(1)
    reqs = int(match.group(2))
    k_val = int(match.group(4))
    
    raw_method = match.group(5)
    method = raw_method.replace('.wcnf', '')
    if method.startswith('sat_'):
        method = method.replace('sat_', 'maxsat_')

    obj = np.nan
    bound = np.nan
    incumbent_time = None
    total_time = None
    is_timeout = False

    with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
        content = f.read()

        # Detect 3600-second time limit termination status
        if re.search(r"Time limit reached", content, re.IGNORECASE) or \
           re.search(r"\[UWrMaxSAT\] Timeout", content) or \
           re.search(r"NO feasible solution", content, re.IGNORECASE) or \
           re.search(r"\[Gurobi.*?\] Status: 9", content):
            is_timeout = True

        # Extract logged incumbent records (Time, Obj, Bound)
        incumbents = re.findall(r"\[Incumbent.*?\]\s*Time:\s*([\d\.]+)s\s*\|\s*Obj:\s*([-\d\.]+)(?:\s*\|\s*Bound:\s*([-\d\.]+))?", content)

        # Extract final primal objective z, dual bound, and total runtime T_total by solver family
        if method.startswith("maxsat"):
            obj_match = re.search(r"\[UWrMaxSAT\] OBJ:\s*([-\d\.]+)", content)
            if obj_match:
                obj = float(obj_match.group(1))

            bound_match = re.search(r"\[UWrMaxSAT\] BOUND:\s*([-\d\.]+)", content)
            if bound_match:
                bound = float(bound_match.group(1))

            time_match = re.search(r"c CPU time\s*:\s*([\d\.]+)\s*s", content)
            if time_match:
                total_time = float(time_match.group(1))
                
            if not re.search(r"OPTIMUM FOUND", content) and not re.search(r"SATISFIABLE", content):
                is_timeout = True

        elif method == "cpsat":
            obj_match = re.search(r"(?:FINAL OBJ:|objective:)\s*([-\d\.]+)", content, re.IGNORECASE)
            if obj_match:
                obj = float(obj_match.group(1))

            bound_match = re.search(r"(?:BEST BOUND:|best bound:)\s*([-\d\.]+)", content, re.IGNORECASE)
            if bound_match:
                bound = float(bound_match.group(1))

            time_match = re.search(r"Total Runtime:\s*([\d\.]+)\s*sec", content, re.IGNORECASE)
            if time_match:
                total_time = float(time_match.group(1))

            if not re.search(r"\[CP-SAT\] Status:\s*(OPTIMAL|INFEASIBLE)", content, re.IGNORECASE):
                is_timeout = True

        elif "hybrid" in method or "full" in method or method in ["static", "mip"]:
            obj_match = re.search(r"\[Gurobi.*?\] BEST OBJ:\s*([-\d\.]+)", content)
            if obj_match:
                obj = float(obj_match.group(1))

            bound_match = re.search(r"\[Gurobi.*?\] BEST BOUND:\s*([-\d\.]+)", content)
            if bound_match:
                bound = float(bound_match.group(1))

            time_match = re.search(r"\[Gurobi.*?\] Runtime:\s*([\d\.]+)\s*sec", content)
            if time_match:
                total_time = float(time_match.group(1))

        # Retrieve objective value from solution evaluation block if not reported in solver header
        eval_match = re.search(r"Objective Value\s*=\s*([-\d\.]+)", content)
        if eval_match:
            eval_obj = float(eval_match.group(1))
            if pd.isna(obj) or (obj == 0.0 and eval_obj > 0.0):
                obj = eval_obj

        # Retrieve dual bound from the final incumbent entry if not reported in termination summary
        if pd.isna(bound) and incumbents:
            last_bound_str = incumbents[-1][2]
            if last_bound_str:
                bound = float(last_bound_str)

        # Determine time to best incumbent T_best (Section VI-C)
        if pd.notna(obj) and obj > 0 and incumbents:
            for t_str, o_str, _ in incumbents:
                if abs(float(o_str) - obj) < 1e-5:
                    incumbent_time = float(t_str)
                    break
        
        if incumbent_time is None and total_time is not None:
            incumbent_time = total_time

    # Normalize floating-point zero representations
    if pd.notna(obj) and obj == 0.0:
        obj = 0.0
    if pd.notna(bound) and bound == 0.0:
        bound = 0.0

    if "hybrid" in method or "full" in method or method in ["static", "mip"]:
        if pd.notna(obj) and obj == 0.0 and pd.notna(bound) and bound == 0.0:
            is_timeout = True

    return {
        "Instance": instance,
        "Requests": reqs,
        "K": k_val,
        "Method": method,
        "HasFeasible": pd.notna(obj),
        "Timeout": is_timeout,
        "Objective": obj,
        "BestBound": bound,
        "Time_to_Best(s)": incumbent_time,
        "Total_Time(s)": total_time
    }

def process_results(log_directory):
    """
    Aggregate all solver logs in log_directory, compute the global Best Known Bound (BKB)
    and relative optimality gap (BKB - z) / z * 100% for each instance, and export to
    ppdsp_results.csv (Section VI-C).
    """
    results = []
    for root, dirs, files in os.walk(log_directory):
        for file in files:
            if file.endswith(".out"):
                filepath = os.path.join(root, file)
                res = parse_log_file(filepath)
                if res:
                    results.append(res)

    df = pd.DataFrame(results)
    if df.empty:
        print("No valid log files found.")
        return

    df = df.sort_values(by=["Instance", "Requests", "K", "Method"])
    df['BKB'] = np.nan
    df['Gap(%)'] = np.nan

    # Compute global BKB and relative optimality gap per benchmark instance (Section VI-C)
    for (inst, req, k_val), group in df.groupby(['Instance', 'Requests', 'K']):
        
        valid_objs = group['Objective'].dropna()
        global_max_obj = valid_objs.max() if not valid_objs.empty else 0.0
        
        # Filter out invalid dual bounds that fall below the highest validated primal objective
        for idx in group.index:
            bb = df.loc[idx, 'BestBound']
            if pd.notna(bb) and bb < global_max_obj - 1e-5:
                df.loc[idx, 'BestBound'] = np.nan
                df.loc[idx, 'Timeout'] = True
                
        valid_bounds = df.loc[group.index, 'BestBound'].dropna()
        
        # Identify runs terminating with proven optimality within 3600 seconds
        optimal_runs = group[(group['Timeout'] == False) & (group['Objective'].notna())]
        trusted_optimals = optimal_runs[optimal_runs['Objective'] >= global_max_obj - 1e-5]
        
        # Define BKB as the proven optimal value or the tightest validated dual upper bound
        if not trusted_optimals.empty:
            bkb = trusted_optimals['Objective'].max()
        else:
            if not valid_bounds.empty:
                bkb = valid_bounds.min()
            else:
                bkb = np.nan
                
        df.loc[group.index, 'BKB'] = bkb
        
        # Compute relative optimality gap: (BKB - z) / z * 100% (evaluates to infinity when z = 0)
        for idx in group.index:
            obj = df.loc[idx, 'Objective']
            if pd.notna(obj) and pd.notna(bkb):
                if obj == 0.0:
                    df.loc[idx, 'Gap(%)'] = np.inf if bkb > 0.0 else 0.0
                else:
                    gap = ((bkb - obj) / obj) * 100.0
                    df.loc[idx, 'Gap(%)'] = max(0.0, round(gap, 2))

    # Set total execution time to 3600.0s for runs reaching the wall-clock time limit
    for idx in df.index:
        if df.loc[idx, 'Timeout']:
            tt = df.loc[idx, 'Total_Time(s)']
            if pd.isna(tt) or tt < 3600.0:
                df.loc[idx, 'Total_Time(s)'] = 3600.0

    cols = ['Instance', 'Requests', 'K', 'Method', 'HasFeasible', 'Timeout', 
            'Objective', 'BestBound', 'BKB', 'Gap(%)', 'Time_to_Best(s)', 'Total_Time(s)']
    df = df[cols]
    
    output_path = "ppdsp_results.csv"
    df.to_csv(output_path, index=False)
    print(f"Results successfully saved to {output_path}")

if __name__ == "__main__":
    process_results("./")
