# Structural Lazy Clause Generation for the Profit-Maximizing Pickup and Delivery Selection Problem

This repository contains the benchmark synthesis scripts, exact solver implementations, and result aggregation tools for the paper **"Structural Lazy Clause Generation for the Profit-Maximizing Pickup and Delivery Selection Problem"**.

The codebase provides a unified evaluation framework over location-based sparse routing networks across eight exact solver configurations spanning three optimization paradigms:
1. **Mixed-Integer Programming (MIP — Gurobi)**:
   - `static`: Monolithic branch-and-cut formulation (Eqs. (1)–(9)) with integer visit sequence variables $u^t_v$ and continuous vehicle load variables $h^t_v$.
   - `hybrid-path`: Retains static sequence variables $u^t_v$ (Eqs. (7)–(8)), omits $h^t_v$ and Eq. (9), and dynamically separates path-based capacity cuts (Eq. (10)) via lazy constraint callbacks.
   - `hybrid-node`: Retains static sequence variables $u^t_v$ (Eqs. (7)–(8)), omits $h^t_v$ and Eq. (9), and dynamically separates structural node-based capacity cuts (Eq. (11)) via lazy constraint callbacks.
   - `full-path`: Omits $u^t_v$, $h^t_v$, and Eqs. (7)–(9), separating subtour elimination cuts, path-based precedence cuts, and path-based capacity cuts (Eq. (10)) via lazy constraint callbacks.
   - `full-node`: Omits $u^t_v$, $h^t_v$, and Eqs. (7)–(9), separating subtour elimination cuts, path-based precedence cuts, and node-based capacity cuts (Eq. (11)) via lazy constraint callbacks.
2. **Constraint Programming with Lazy Clause Generation (CP — Google OR-Tools CP-SAT)**:
   - `CP-SAT`: Global circuit propagator enforcing flow conservation and subtour elimination over active vertices, coupled with conditional integer constraints for visit ordering and load accumulation.
3. **Weighted Partial Maximum Satisfiability with Structural LCG (MaxSAT — UWrMaxSat)**:
   - `MaxSAT-path`: Propositional encoding with topological order literals $\nu^t_{d,\epsilon}$ (Eqs. (12)–(19)) and Counterexample-Guided Abstraction Refinement (CEGAR) separating path-based LCG clauses (Eq. (20)).
   - `MaxSAT-order`: Propositional encoding with topological order literals $\nu^t_{d,\epsilon}$ (Eqs. (12)–(19)) and CEGAR separating structural order-based LCG clauses (Eq. (21)).

---

## 1. Environment and Dependencies

### Python Environment
Ensure Python 3.8+ is installed. Install the required Python packages using `pip`:

```bash
pip install -r requirements.txt
```

The computational experiments reported in the paper were conducted using the following solver and library versions:
- **`gurobipy`**: Gurobi Optimizer 13.0.3 (requires an active Gurobi license)
- **`ortools`**: Google OR-Tools 9.15 (`CP-SAT` solver)
- **`python-sat`**: PySAT toolkit (`WCNF`, `CNF`, and `IDPool` formula encoding)
- **`networkx`**: Graph topology and Euclidean Minimum Spanning Tree (MST) construction
- **`pandas`** & **`numpy`**: Data manipulation, log parsing, and statistical aggregation

### External MaxSAT Solver Compilation (`UWrMaxSat`)
The MaxSAT configurations (`MaxSAT-path` and `MaxSAT-order`) rely on a modified C++ build of **`UWrMaxSat`** that embeds the CEGAR-based structural LCG procedure (Algorithm 1, implemented in `PPDSP_utils.cc`) directly into the core-guided search loop.

A build script is provided to compile `uwrmaxsat` alongside its required libraries (COMiniSatPS, MaxPre, and SCIP) and register the binary on your system `PATH`. From the repository root, execute:

```bash
cd UWrMaxSat-SCIP-MaxPre
chmod +x code/starexec_build
./code/starexec_build
source ~/.bashrc
```

*(Note: During experimental evaluation, `ppdsp_maxsat.py` invokes `uwrmaxsat` with the `-no-scip` flag so that all bounds and truth assignments are derived exclusively through core-guided Boolean reasoning.)*

---

## 2. Repository Structure

### Core Benchmark & Solver Scripts
- **`ppdsp_ins_arg.py`**: Parses root CVRPLib (`.vrp`) instances (Sets A, B, and P), executes two-phase empirical demand sampling up to $1.5 \cdot |T| \cdot c$ (using fixed random seed `42`), computes request revenues $p_r$, constructs directed sparse graphs $G=(V, E)$ via the union of the Euclidean MST, symmetric $k$-Nearest Neighbor ($k$-NN) edges ($k \in \{3, 4, 5\}$), and bidirectional depot arcs, and exports solver-ready CSV files (Section VI-A).
- **`ppdsp_ins_gen.py`**: Reads synthesized CSV files (`2DNode_*.csv`, `adjMatrx*_.csv`, `requestInfo*_.csv`, `vehicleInfo*_.csv`), computes all-pairs shortest-path travel costs $l_{od}$ via the Floyd-Warshall algorithm, and allocates global variable indices for $x^t_{od}$, $y^t_r$, $\nu^t_{d,\epsilon}$, $u^t_v$, and $h^t_v$ (Section III-A, Section V-C, Section VI-A).
- **`ppdsp_utils.py`**: Provides truth-assignment decoding, ordered route reconstruction $\mathcal{R}^t$, sequential capacity simulation and greedy Minimal Conflict Set (MCS) extraction (Algorithm 1), homogeneous fleet symmetry grouping, net operational profit evaluation (Eq. (1)), and `.meta` file export for `UWrMaxSat` (Section III-B, Section IV).
- **`ppdsp_mip.py`**: Implements the Gurobi MIP model and lazy constraint callbacks supporting the `static`, `hybrid-path`, `hybrid-node`, `full-path`, and `full-node` configurations (Section III-B Eqs. (1)–(9), Section IV Eqs. (10)–(11), Section VI-B1).
- **`ppdsp_cpsat.py`**: Implements the Google OR-Tools `CP-SAT` formulation using the global `AddCircuit` propagator, conditional integer precedence/load constraints, and lexicographic symmetry breaking (Section VI-B2).
- **`ppdsp_maxsat.py`**: Encodes the PPDSP as a weighted partial MaxSAT instance $\Phi = (\mathcal{C}_{\text{hard}}, \mathcal{C}_{\text{soft}})$ using weighted soft clauses (Section V-A), pairwise At-Most-One clauses (Eqs. (12)–(13)), visitation and flow implications (Eqs. (14)–(16)), topological order clauses (Eqs. (17)–(19)), and propositional symmetry breaking (Section V-D), and interfaces with `uwrmaxsat` (Section VI-B3).
- **`UWrMaxSat-SCIP-MaxPre/`** (containing **`PPDSP_utils.cc`**): C++ source tree of `UWrMaxSat` implementing in-memory candidate route decoding, greedy MCS reduction (Algorithm 1, Proposition 1), and symmetry-broadcast clause injection for path-based LCG (Eq. (20)) and order-based LCG (Eq. (21)) (Section V-D, Section VI-B3).

### Execution, Post-Processing & Visualization Scripts
- **`ppdsp_main.py`**: Unified command-line interface for running an individual benchmark instance under a specified solver configuration (Section VI-B).
- **`ppdsp_run_exp.py`**: Automated batch orchestrator that synthesizes all 222 PPDSP benchmark instances (74 root CVRPLib instances across $k \in \{3, 4, 5\}$) and dispatches all eight exact solver configurations under a 3600-second wall-clock time limit and single-thread execution, followed by automatic compilation of results (Section VI-A, Section VI-B).
- **`ppdsp_results.py`**: Parses all generated `.out` solver logs, validates dual upper bounds, computes the global Best Known Bound (BKB) and relative optimality gap $(\text{BKB} - z)/z \times 100\%$, and exports the master dataset `ppdsp_results.csv` (Section VI-C).
- **`make_latex_tables.py`** & **`make_supplementary.py`**: Post-processing scripts that read `ppdsp_results.csv` to generate the solution quality dominance summary table (`section6_table.tex`, Table II) and the standalone 38-table instance-by-instance supplementary document (`appendix_tables.tex` and `supplementary.tex`).
- **`figDataClas.py`**: Post-processing script that reads `ppdsp_results.csv`, partitions instances by benchmark group, and computes the empirical cumulative distribution function (ECDF) step-plot coordinates of relative optimality gaps across all 222 instances (`ppdsp_all_gap.csv`) and within individual sets (`ppdsp_A_gap.csv`, `ppdsp_B_gap.csv`, `ppdsp_P_gap.csv`) (Section VI-D3).
- **`plot_gap.tex`**: Standalone TikZ/PGFPlots LaTeX file that compiles `ppdsp_all_gap.csv`, `ppdsp_A_gap.csv`, `ppdsp_B_gap.csv`, and `ppdsp_P_gap.csv` into the 1x4 relative optimality gap profile figure (`plot_gap.pdf`, Fig. 1 in Section VI-D3).

---

## 3. Usage Guide

### Step 1: Benchmark Instance Synthesis
To convert a raw CVRPLib `.vrp` file into solver-ready CSV matrices for a specific $k$-NN connectivity density:

```bash
# Usage: python ppdsp_ins_arg.py <file.vrp> [outDir] [seed] [k_nn]
python ppdsp_ins_arg.py P-n22-k8.vrp . 42 3
```

This command extracts the fleet size $|T|$ and vehicle capacity $c$, samples pickup-and-delivery requests up to cumulative demand $1.5 \cdot |T| \cdot c$ with random seed `42`, and generates:
- `2DNode_P-n22-k8.csv`: Normalized 2D node coordinates (customers indexed $0 \dots |V|-2$, depot at $|V|-1$).
- `adjMatrx3_P-n22-k8.csv`: Directed adjacency matrix for $k\text{-NN} = 3$ unified with the Euclidean MST and depot arcs.
- `vehicleInfo8_P-n22-k8.csv`: Homogeneous vehicle capacity and cost parameters.
- `requestInfo<|R|>_P-n22-k8.csv`: Synthesized request tuples `[p_r, q_r, f(r), g(r)]` (e.g., `requestInfo29_P-n22-k8.csv`).

### Step 2: Single Instance Evaluation
Use `ppdsp_main.py` to evaluate a single instance under any of the eight exact configurations. The script automatically detects the fleet size from `vehicleInfo*_<instance>.csv`.

```bash
# 1. MIP — static configuration (Eqs. (1)-(9))
python ppdsp_main.py mip P-n22-k8 29 --knn 3 --mip_strategy static --time 3600

# 2. MIP — hybrid-path configuration (MTZ + path-based capacity cuts in Eq. (10))
python ppdsp_main.py mip P-n22-k8 29 --knn 3 --mip_strategy hybrid --cut_type path --time 3600

# 3. MIP — hybrid-node configuration (MTZ + node-based capacity cuts in Eq. (11))
python ppdsp_main.py mip P-n22-k8 29 --knn 3 --mip_strategy hybrid --cut_type node --time 3600

# 4. MIP — full-path configuration (pure lazy subtour, precedence, and Eq. (10) capacity cuts)
python ppdsp_main.py mip P-n22-k8 29 --knn 3 --mip_strategy full --cut_type path --time 3600

# 5. MIP — full-node configuration (pure lazy subtour, precedence, and Eq. (11) capacity cuts)
python ppdsp_main.py mip P-n22-k8 29 --knn 3 --mip_strategy full --cut_type node --time 3600

# 6. CP — Google OR-Tools CP-SAT configuration
python ppdsp_main.py cpsat P-n22-k8 29 --knn 3 --time 3600

# 7. MaxSAT — MaxSAT-path configuration (Order Encoding + path-based LCG in Eq. (20))
python ppdsp_main.py sat P-n22-k8 29 --knn 3 --cut_type path --time 3600

# 8. MaxSAT — MaxSAT-order configuration (Order Encoding + order-based LCG in Eq. (21))
python ppdsp_main.py sat P-n22-k8 29 --knn 3 --cut_type order --time 3600
```

**Command-Line Arguments for `ppdsp_main.py`:**
- `solver`: Solver paradigm (`mip`, `cpsat`, `sat`).
- `instance`: Root CVRPLib instance name without `.vrp` extension (e.g., `A-n32-k5`, `B-n31-k5`, `P-n22-k8`).
- `reqs`: Number of requests $|R|$ in the synthesized `requestInfo<|R|>_<instance>.csv`.
- `--knn`: $k$-NN graph connectivity parameter $k \in \{3, 4, 5\}$ (default: `3`).
- `--mip_strategy`: MIP formulation strategy (`static`, `hybrid`, `full`; default: `hybrid`).
- `--cut_type`: Structural capacity cut type (`path` or `node` for `mip`; `path` or `order` for `sat`; default: `path`).
- `--time`: Wall-clock time limit in seconds (default: `3600`).
- `--vehs` / `--cap`: Optional overrides for fleet size $|T|$ and vehicle capacity $c$.

### Step 3: Reproducing All Paper Experiments, Tables, and Figures
To reproduce the complete benchmark evaluation across all 222 instances ($74 \text{ root instances} \times 3 \text{ connectivity densities } k \in \{3, 4, 5\}$) and all eight solver configurations ($1,776$ total solver runs):

1. Place the 74 root CVRPLib `.vrp` files (Sets A, B, and P) in the working directory.
2. Adjust `MAX_WORKERS` in `ppdsp_run_exp.py` to match the number of available physical CPU cores on your machine (each solver task is strictly pinned to 1 thread).
3. Run the batch experiment script directly (or in the background via `nohup`):

```bash
# Direct execution
python ppdsp_run_exp.py

# Or background execution with progress logging
nohup python ppdsp_run_exp.py > master_ppdsp.log 2>&1 &
tail -f master_ppdsp.log
```

`ppdsp_run_exp.py` executes the entire workflow automatically:
- Synthesizes the CSV instance files for all 74 root instances across $k \in \{3, 4, 5\}$ with `GLOBAL_SEED = 42`.
- Executes all eight solver configurations (`static`, `hybrid_path`, `hybrid_node`, `full_path`, `full_node`, `cpsat`, `maxsat_path`, `maxsat_order`) with a 3600-second time limit per run, writing detailed trajectories and incumbent logs to `.out` files.
- Invokes `python ppdsp_results.py` upon completion to parse all `.out` logs and generate `ppdsp_results.csv`.

4. To re-aggregate `.out` logs or generate the LaTeX tables and optimality gap figure (Fig. 1) from `ppdsp_results.csv` at any time:

```bash
# 1. Parse .out solver logs into ppdsp_results.csv
python ppdsp_results.py

# 2. Compute ECDF optimality gap coordinates (ppdsp_*_gap.csv) and compile Fig. 1 (plot_gap.pdf)
python figDataClas.py
pdflatex plot_gap.tex

# 3. Generate LaTeX summary table (section6_table.tex) and 38-table supplementary document (appendix_tables.tex, supplementary.tex)
python make_latex_tables.py
python make_supplementary.py
pdflatex supplementary.tex
```

---

## 4. License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
