# ppdsp_main.py
# Command-line entry point for executing individual PPDSP exact solver configurations (Section VI-B).

import sys
import os
import glob
import argparse

from ppdsp_mip import PPDSP_MIP
from ppdsp_maxsat import PPDSP_MaxSAT
from ppdsp_cpsat import PPDSP_CPSAT

def main():
    """
    Parse command-line arguments and dispatch the selected exact solver configuration:
    - MIP configurations (Section VI-B1): static, hybrid-path, hybrid-node, full-path, full-node
    - CP configuration (Section VI-B2): CP-SAT
    - MaxSAT configurations (Section VI-B3): MaxSAT-path, MaxSAT-order
    """
    parser = argparse.ArgumentParser(description="Exact Solvers Testbed for Profit-Maximizing PDP (PPDSP)")
    parser.add_argument("solver", choices=["mip", "sat", "cpsat"], help="Choose the exact solver")
    parser.add_argument("--mip_strategy", type=str, choices=['static', 'hybrid', 'full'], default='hybrid', 
                    help="Choose Benders Cut strategy for MIP: 'static' (Pure Static), 'hybrid' (MTZ+Cap LCG), or 'full' (Pure Lazy)")
    parser.add_argument('--cut_type', type=str, default='path', choices=['node', 'path', 'order'], 
                    help='Capacity cut strategy: node/path for MIP, order/path for MaxSAT')
    parser.add_argument("instance", type=str, help="Benchmark name without extension (e.g., P-n22-k8)")
    parser.add_argument("reqs", type=int, help="Number of requests to test (e.g., 35)")
    parser.add_argument("--vehs", type=int, default=None, help="Override benchmark global fleet size (optional)")
    parser.add_argument("--cap", type=int, default=None, help="Override benchmark global capacity (optional)")
    parser.add_argument("--knn", type=int, default=3, help="k-NN sparsification factor (default: 3)")
    parser.add_argument("--time", type=int, default=3600, help="Time limit in seconds (default: 3600)")

    if len(sys.argv) == 1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    args = parser.parse_args()

    ppdsp_name = args.instance.replace('.vrp', '')
    
    # Automatically detect fleet size |T| from the generated vehicleInfo CSV file if not overridden
    num_vehs = args.vehs
    if num_vehs is None:
        pattern = f"vehicleInfo*_{ppdsp_name}.csv"
        matching_files = glob.glob(pattern)
        if matching_files:
            try:
                filename = os.path.basename(matching_files[0])
                num_vehs_str = filename.replace("vehicleInfo", "").split("_")[0]
                num_vehs = int(num_vehs_str)
            except ValueError:
                pass

    if num_vehs is None:
        print(f"[Error] Could not automatically detect vehicle count for {ppdsp_name}. Please specify --vehs.")
        sys.exit(1)
        
    capacity = args.cap if args.cap is not None else -1 

    print("\n==================================================")
    print(f"  PPDSP Exact Solver: {args.solver.upper()}")
    print(f"  Instance: {ppdsp_name} | Requests: {args.reqs}")
    print(f"  Fleet: {num_vehs} Vehicles | Fallback Cap: {'CSV Data' if capacity == -1 else capacity}")
    print(f"  Graph: {args.knn}-NN | Time Limit: {args.time}s")
    if args.solver == "mip":
        print(f"  MIP Strategy: {args.mip_strategy.upper()}")
    print("==================================================\n")

    if args.solver == "mip":
        solver = PPDSP_MIP(
            pdpName=ppdsp_name, 
            num_reqs=args.reqs, 
            num_vehs=num_vehs, 
            capacity=capacity, 
            knn=args.knn, 
            bc_strategy=args.mip_strategy, 
            cut_type=args.cut_type
        )
        solver.solve(time_limit=args.time)

    elif args.solver == "sat":
        solver = PPDSP_MaxSAT(
            pdpName=ppdsp_name, 
            num_reqs=args.reqs, 
            num_vehs=num_vehs, 
            capacity=capacity, 
            knn=args.knn, 
            cut_type=args.cut_type
        )
        solver.genMaxsatFormular()
        solver.solve(time_limit=args.time)

    elif args.solver == "cpsat":
        solver = PPDSP_CPSAT(
            pdpName=ppdsp_name, 
            num_reqs=args.reqs, 
            num_vehs=num_vehs, 
            capacity=capacity, 
            knn=args.knn
        )
        solver.solve(time_limit=args.time)

if __name__ == "__main__":
    main()
