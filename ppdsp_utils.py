# ppdsp_utils.py
# Model decoding, route reconstruction, objective evaluation, and solver metadata export
# (Section III-A, Section IV, Section V).

import os

class GlobalVariableRegistry:

    # ==========================================
    # Global variable index registry
    # ==========================================
    def __init__(self):
        self.varCounter = 0
        self.varDict = {}

    def get_id(self, var_type, *args):
        """
        Retrieve or allocate a unique global variable identifier for decision variable
        types ('x', 'y', 'u', 'nu', 'h') indexed by their tuple arguments (Section III-A, Section V-C).
        """
        key = (var_type,) + args
        if key not in self.varDict:
            self.varCounter += 1
            self.varDict[key] = self.varCounter
        return self.varDict[key]

    def get_max_core_id(self):
        """
        Return the highest variable identifier currently allocated in the registry.
        """
        return self.varCounter


class PPDSP_utils:

    # ----------------------------
    # Parse solver output log to extract core assignment literals
    # ----------------------------
    @staticmethod
    def parse_and_save_assumption(log_file, assumption_file, lastY, mode):
        if not os.path.exists(log_file):
            print(f"  [Warning] Log file {log_file} not found.")
            return False

        core_lits = []
        
        if mode == "maxsat":
            with open(log_file, "r") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("v "):
                        lits = line.split()[1:]
                        for lit_str in lits:
                            try:
                                lit = int(lit_str)
                                if abs(lit) <= lastY:
                                    core_lits.append(lit)
                            except ValueError:
                                pass
        elif mode == "mip":
            positive_vars = set()
            raw_model_found = False
            with open(log_file, "r") as f:
                lines = f.readlines()
                for i, line in enumerate(lines):
                    if line.startswith("===== RAW XY MODEL ====="):
                        raw_model_found = True
                        if i + 1 < len(lines):
                            val_str = lines[i+1].strip()
                            if val_str:
                                positive_vars = set(int(v) for v in val_str.split())
                        break
            
            if raw_model_found:
                for vid in range(1, lastY + 1):
                    if vid in positive_vars:
                        core_lits.append(vid)
                    else:
                        core_lits.append(-vid)
        
        if core_lits:
            with open(assumption_file, "w") as f:
                f.write(" ".join(map(str, core_lits)) + "\n")
            print(f"  [Info] Saved {len(core_lits)} core assumption literals to {assumption_file}")
            return True
        else:
            print("  [Warning] No valid literals found in log. Assumption creation failed.")
            return False

    # ----------------------------
    # Map variable name strings to integer literal identifiers
    # ----------------------------
    @staticmethod
    def convert_model(raw_model):
        filtered = []
        for name in raw_model:
            if len(name) > 1 and (name[0] == 'x' or name[0] == 'y'):
                try:
                    filtered.append(int(name[1:]))
                except Exception:
                    pass
        return filtered

    # ----------------------------
    # Construct inverse index map: variable ID -> ('x', t, o, d) or ('y', r, t)
    # ----------------------------
    @staticmethod
    def buildVarIndexMap(self):
        self.id2Var = {}
        for t in range(self.lenOfVehicle):
            for o in range(len(self.xVarList[t])):
                for d in range(len(self.xVarList[t][o])):
                    vid = self.xVarList[t][o][d]
                    self.id2Var[vid] = ('x', t, o, d)
        
        for r in range(self.lenOfRequest):
            for t in range(self.lenOfVehicle):
                vid = self.yVarList[r][t]
                self.id2Var[vid] = ('y', r, t)

    # ----------------------------
    # Filter truth assignment to retain positive routing and selection variables (x^t_{od}, y^t_r)
    # ----------------------------
    @staticmethod
    def extractXYModel(self, model):
        return [i for i in model if 0 < i <= self.getLastYVarID()]

    # ----------------------------
    # Reconstruct ordered vehicle routes R^t and selected request sets S from truth assignments (Section IV-A)
    # ----------------------------
    @staticmethod
    def decodeModel(self, filtered_model):
        if self.id2Var is None:
            PPDSP_utils.buildVarIndexMap(self)

        veh_routes = {v: {'route': [], 'requests': []} for v in range(self.lenOfVehicle)}

        for vid in filtered_model:
            varInfo = self.id2Var.get(vid)
            if varInfo is None:
                continue
            if varInfo[0] == 'x':
                _, t, o, d = varInfo
                if o != d:
                    veh_routes[t]['route'].append((o, d))
            elif varInfo[0] == 'y':
                _, r, t = varInfo
                veh_routes[t]['requests'].append(r)

        for v in range(self.lenOfVehicle):
            edges = veh_routes[v]['route']
            if not edges:
                continue
            next_map = {o: d for (o, d) in edges}
            route = []
            cur = self.lenOfLocation
            while cur in next_map:
                nxt = next_map[cur]
                route.append((cur, nxt))
                cur = nxt
                if cur == self.lenOfLocation:
                    break
            veh_routes[v]['route'] = route
            
        return veh_routes

    # ----------------------------
    # Partition fleet T into homogeneous vehicle symmetry groups (Section IV)
    # ----------------------------
    @staticmethod
    def get_sbc_groups(vehicleList):
        groups = {}
        for t, veh in enumerate(vehicleList):
            cap = int(veh[0])
            cost = float(f"{veh[1]:.4f}")
            key = (cap, cost)
            
            if key not in groups:
                groups[key] = []
            groups[key].append(t)
            
        return {k: v for k, v in groups.items() if len(v) >= 2}

    # ----------------------------
    # Sequential capacity evaluation and Minimal Conflict Set (MCS) extraction (Algorithm 1, Section IV-A & IV-B)
    # ----------------------------
    @staticmethod
    def checkOverload(self, vehID, route, assigned_reqs):
        if not route:
            return False, []

        capacity = self.vehicleList[vehID][0]
        load = 0
        onboard = set()

        # Request parameters [p_r, q_r, f(r), g(r)] for all r in R (Section III-A)
        req_size    = {r: self.requestList[r][1] for r in range(self.lenOfRequest)}
        pickup_node = {r: self.requestList[r][2] for r in range(self.lenOfRequest)}
        drop_node   = {r: self.requestList[r][3] for r in range(self.lenOfRequest)}

        violated = False
        learnt_clause = []

        # Simulate route R^t and track accumulated load Gamma and onboard set S_onboard (Algorithm 1, Lines 2-12)
        for k, (o, d) in enumerate(route):
            for r in assigned_reqs:
                if d == pickup_node[r] and d == drop_node[r]:
                    continue
                elif d == pickup_node[r]:
                    load += req_size[r]
                    onboard.add(r)
                elif d == drop_node[r] and r in onboard:
                    load -= req_size[r]
                    onboard.remove(r)

            # Detect capacity overload Gamma > c and isolate MCS S_conflict via greedy reduction (Algorithm 1, Lines 13-20)
            if load > capacity:
                violated = True
                onboard_reqs = list(onboard)
                onboard_reqs.sort(key=lambda r: req_size[r], reverse=True)

                minimal_conflict = []
                current_subset_load = 0
                for r in onboard_reqs:
                    current_subset_load += req_size[r]
                    minimal_conflict.append(r)
                    if current_subset_load > capacity:
                        break 

                yLits = [-self.yVarList[r][vehID] for r in minimal_conflict]
                prefix_origins = [route[i][0] for i in range(k + 1)]

                xLits = []
                for r in minimal_conflict:
                    dp = drop_node[r]
                    for p in prefix_origins:
                        xLits.append(self.xVarList[vehID][p][dp])

                learnt_clause = yLits + xLits
                break

        return violated, learnt_clause

    # ----------------------------
    # Format and log reconstructed vehicle trajectories R^t and load profiles
    # ----------------------------
    @staticmethod
    def printVehRoutes(self, filtered_model, log_file=None, show_load=True):
        vehRoutes = PPDSP_utils.decodeModel(self, filtered_model)
        depot = self.lenOfLocation
        output_lines = []
        
        for vehID, info in vehRoutes.items():
            route = info['route']
            reqs  = info['requests']
            
            if not route:
                output_lines.append(f"Vehicle {vehID}: Depot, (requests = {reqs})")
                continue
                
            node_seq = [route[0][0]] + [d for (_, d) in route]
            
            node_seq_str = []
            current_load = 0
            
            picked_up = set()
            dropped_off = set()
            
            for n in node_seq:
                name_str = "Depot" if n == depot else str(n)
                
                if show_load:
                    if n != depot:
                        for r in reqs:
                            demand = int(self.requestList[r][1])
                            pickup_node = int(self.requestList[r][2])
                            dropoff_node = int(self.requestList[r][3])
                            
                            if n == pickup_node and r not in picked_up:
                                current_load += demand
                                picked_up.add(r)
                                
                            if n == dropoff_node and r in picked_up and r not in dropped_off:
                                current_load -= demand
                                dropped_off.add(r)
                                
                    node_seq_str.append(f"{name_str} {{{current_load}}}")
                else:
                    node_seq_str.append(name_str)
            
            pretty_route = " → ".join(node_seq_str)
            output_lines.append(f"Vehicle {vehID}: {pretty_route}, (requests = {reqs})")
            
        output_str = "\n".join(output_lines)
        print(output_str)
        
        if log_file and os.path.exists(log_file):
            with open(log_file, "a", encoding="utf-8") as f:
                f.write("\n" + output_str + "\n")

    # ----------------------------
    # Evaluate net operational profit objective in Eq. (1)
    # ----------------------------
    @staticmethod
    def evaluateSolution(self, filtered_model, log_file=None):
        if self.id2Var is None:
            PPDSP_utils.buildVarIndexMap(self)

        cost = 0
        profit = 0

        for vid in filtered_model:
            varInfo = self.id2Var.get(vid)
            if varInfo is None:
                continue

            if varInfo[0] == 'x':
                _, t, o, d = varInfo
                if o != d:
                    cost += self.my_round_int(self.vehicleList[t][1] * self.locaList[o][d])
            elif varInfo[0] == 'y':
                _, r, t = varInfo
                profit += self.requestList[r][0]

        obj = profit - cost
        
        eval_lines = [
            "======== PPDSP SOLUTION EVALUATION ========",
            f"Total Collected Profit = {profit}",
            f"Total Routing Cost     = {cost}",
            f"Objective Value        = {obj}",
            "==========================================="
        ]
        eval_str = "\n".join(eval_lines)
        print(eval_str)
        
        if log_file and os.path.exists(log_file):
            with open(log_file, "a", encoding="utf-8") as f:
                f.write(eval_str + "\n")

        return obj

    # ----------------------------
    # Export variable index tables and instance metadata (.meta) for UWrMaxSat LCG callbacks (Section V-D, Section VI-B3)
    # ----------------------------
    @staticmethod
    def export_meta(self, filename):
        with open(filename, "w") as f:
            f.write(f"{self.lenOfVehicle} {self.lenOfRequest} {self.lenOfLocation}\n")

            f.write("# xVarList\n")
            for t in range(self.lenOfVehicle):
                for o in range(len(self.xVarList[t])):
                    for d in range(len(self.xVarList[t][o])):
                        vid = self.xVarList[t][o][d]
                        f.write(f"{t} {o} {d} {vid}\n")

            f.write("# yVarList\n")
            for r in range(self.lenOfRequest):
                for t in range(self.lenOfVehicle):
                    vid = self.yVarList[r][t]
                    f.write(f"{r} {t} {vid}\n")

            f.write("# nuVarList\n")
            if hasattr(self, 'nuVarList'): 
                for t in range(len(self.nuVarList)):
                    for d in range(len(self.nuVarList[t])):
                        for eps in range(len(self.nuVarList[t][d])):
                            vid = self.nuVarList[t][d][eps]
                            if vid != 0: 
                                f.write(f"{t} {d} {eps} {vid}\n")

            f.write("# requestList\n")
            for r in range(self.lenOfRequest):
                # Export request tuple [p_r, q_r, f(r), g(r)] (Section III-A)
                profit, q, pk, dp = self.requestList[r]
                f.write(f"{r} {profit} {q} {pk} {dp}\n")

            f.write("# vehicleList\n")
            for t in range(self.lenOfVehicle):
                cap, cost = self.vehicleList[t]
                f.write(f"{t} {cap} {cost}\n")

            f.write("# vehicleGroups\n")
            groups = PPDSP_utils.get_sbc_groups(self.vehicleList)
            gid = 0
            for key, vehs in groups.items():
                line = f"{gid} {len(vehs)} " + " ".join(map(str, vehs))
                f.write(line + "\n")
                gid += 1
            
            print(f"[UWrMaxSAT] PPDSP Metadata successfully exported to {filename}.")

    # ----------------------------
    # Load assumption literals from file
    # ----------------------------
    @staticmethod
    def read_assumption_literals(filename):
        lits = set()
        try:
            with open(filename, 'r') as f:
                content = f.read()
                tokens = content.replace('\n', ' ').split()
                for t in tokens:
                    try:
                        lit = int(t)
                        if lit != 0: lits.add(lit) 
                    except ValueError:
                        pass 
        except FileNotFoundError:
            print(f"[Utils] Assumption file not found: {filename}")
        return lits
