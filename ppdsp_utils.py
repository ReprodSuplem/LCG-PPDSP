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
    # Verify route feasibility (Eqs. (2)-(9)) and evaluate net operational profit objective in Eq. (1)
    # ----------------------------
    @staticmethod
    def evaluateSolution(self, filtered_model, log_file=None):
        if self.id2Var is None:
            PPDSP_utils.buildVarIndexMap(self)

        # Independent feasibility verification across all vehicles and assigned requests
        vehRoutes = PPDSP_utils.decodeModel(self, filtered_model)
        active_edges_per_veh = {t: [] for t in range(self.lenOfVehicle)}
        for vid in filtered_model:
            varInfo = self.id2Var.get(vid)
            if varInfo is not None and varInfo[0] == 'x':
                _, t, o, d = varInfo
                if o != d:
                    active_edges_per_veh[t].append((o, d))

        served_reqs = set()
        for t, info in vehRoutes.items():
            route = info['route']
            reqs = info['requests']
            cap = self.vehicleList[t][0]
            assert len(route) == len(active_edges_per_veh[t]), (
                f"Disconnected subtour or degree violation on vehicle {t}: "
                f"decoded route has {len(route)} arcs vs {len(active_edges_per_veh[t])} active arcs"
            )
            if route:
                assert route[0][0] == self.lenOfLocation and route[-1][1] == self.lenOfLocation, (
                    f"Route on vehicle {t} does not start and end at the depot ({self.lenOfLocation})"
                )
            visited_order = {}
            for step_idx, (u, v) in enumerate(route):
                assert self.adjMatrx[u][v] != 0, f"Invalid non-adjacent arc ({u},{v}) on vehicle {t}"
                if v != self.lenOfLocation:
                    assert v not in visited_order, f"Duplicate visit to customer node {v} on vehicle {t}"
                    visited_order[v] = step_idx
            for r in reqs:
                assert r not in served_reqs, f"Request {r} assigned to multiple vehicles"
                served_reqs.add(r)
                pk, dp = self.requestList[r][2], self.requestList[r][3]
                assert pk in visited_order and dp in visited_order, (
                    f"Unvisited pickup/delivery endpoint ({pk} -> {dp}) for request {r} on vehicle {t}"
                )
                if pk != dp:
                    assert visited_order[pk] < visited_order[dp], (
                        f"Precedence violated for request {r} on vehicle {t}: pickup {pk} after delivery {dp}"
                    )
            load = 0
            onboard = set()
            for _, v in route:
                if v != self.lenOfLocation:
                    for r in reqs:
                        if v == self.requestList[r][2] and v != self.requestList[r][3]:
                            load += self.requestList[r][1]
                            onboard.add(r)
                        elif v == self.requestList[r][3] and r in onboard:
                            load -= self.requestList[r][1]
                            onboard.remove(r)
                    assert load <= cap, f"Capacity exceeded ({load} > {cap}) at node {v} on vehicle {t}"

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
