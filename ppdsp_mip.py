# ppdsp_mip.py
# Mixed-Integer Programming (MIP) formulations and branch-and-cut lazy constraint callbacks
# (Section III-B, Section IV, Section VI-B1).

import os
import time
import gurobipy as gp
from gurobipy import GRB
from ppdsp_ins_gen import PPDSP_reform
from ppdsp_utils import PPDSP_utils

class PPDSP_MIP(PPDSP_reform):
    """
    MIP solver supporting five exact branch-and-cut configurations (Section VI-B1):
    - static: Monolithic formulation in Eqs. (1)-(9) with sequence variables u^t_v and load variables h^t_v.
    - hybrid-path / hybrid-node: Retains u^t_v in Eqs. (7)-(8), omits h^t_v and Eq. (9), and separates
      path-based cuts in Eq. (10) or node-based cuts in Eq. (11) via lazy constraint callbacks.
    - full-path / full-node: Omits u^t_v, h^t_v, and Eqs. (7)-(9) from the initial model, separating
      subtour elimination cuts, path-based precedence cuts, and capacity cuts in Eq. (10) or Eq. (11)
      via lazy constraint callbacks.
    """
    def __init__(self, pdpName, num_reqs, num_vehs, capacity, knn, increment=None, bc_strategy='hybrid', cut_type='path'):
        super().__init__(pdpName, num_reqs, num_vehs, capacity, knn, increment)
        self.knn = int(knn)
        self.bc_strategy = bc_strategy.lower()
        self.cut_type = cut_type.lower()

        self.env = gp.Env(empty=True)
        self.env.setParam("OutputFlag", 1)
        self.env.setParam("LogFile", "")
        self.env.start()
        
        # Configure instance identifier according to MIP strategy and capacity cut type (Section VI-B1)
        strategy_suffix = "static" if self.bc_strategy == 'static' else f"{self.bc_strategy}_{self.cut_type}"
        self.insName = f"ppdsp_{pdpName}_r{num_reqs}v{num_vehs}k{knn}_{strategy_suffix}"
        self.model = gp.Model(self.insName, env=self.env)
        
        self.x = {} 
        self.y = {} 
        self.u = {}
        self.h = {}
        
    def genMipModel(self):
        self.genXVarList()
        self.genYVarList()

        if self.bc_strategy in ['hybrid', 'static']:
            self.genUVarList()
        if self.bc_strategy == 'static':
            self.genHVarList()

        self.DEPOT = self.lenOfLocation
        num_nodes = 1 + self.lenOfLocation
        num_reqs = self.lenOfRequest
        num_vehs = self.lenOfVehicle
        n = self.lenOfLocation

        # ==================== Decision Variable Declarations (Section III-A) ====================
        cap = self.vehicleList[0][0]

        for t in range(num_vehs):
            for r in range(num_reqs):
                # Binary request assignment variables y^t_r in {0, 1}
                var_id = self.yVarList[r][t]
                self.y[r, t] = self.model.addVar(vtype=GRB.BINARY, name=f"y{var_id}")
                
            for i in range(num_nodes):
                # Sequence variables 0 <= u^t_v <= |V|-2 for customer locations v in V \ {0}
                if self.bc_strategy in ['static', 'hybrid'] and i < self.DEPOT:
                    u_id = self.uVarList[t][i]
                    self.u[t, i] = self.model.addVar(lb=0, ub=n-1, vtype=GRB.CONTINUOUS, name=f"u{u_id}")
                
                # Continuous vehicle load variables 0 <= h^t_v <= c
                if self.bc_strategy == 'static':
                    h_id = self.hVarList[t][i]
                    self.h[t, i] = self.model.addVar(lb=0, ub=cap, vtype=GRB.CONTINUOUS, name=f"h{h_id}")
                    
                # Binary routing variables x^t_{od} in {0, 1}
                for j in range(num_nodes):
                    if i == j:
                        var_id = self.xVarList[t][i][i]
                        self.x[t, i, i] = self.model.addVar(vtype=GRB.BINARY, name=f"x{var_id}")
                    elif self.adjMatrx[i][j] != 0:
                        var_id = self.xVarList[t][i][j]
                        self.x[t, i, j] = self.model.addVar(vtype=GRB.BINARY, name=f"x{var_id}")
                    else:
                        self.x[t, i, j] = None

        self.model.update()

        # ==================== Objective Function: Eq. (1) ====================
        obj_expr = gp.LinExpr()
        
        # 1. Total revenue collected from served requests: sum_{t in T} sum_{r in R} p_r y^t_r
        for r in range(num_reqs):
            profit = self.requestList[r][0]
            for t in range(num_vehs):
                obj_expr.addTerms(profit, self.y[r, t])
                
        # 2. Total fleet routing cost: - sum_{t in T} sum_{(o,d) in E} l_{od} x^t_{od}
        for t in range(num_vehs):
            veh_cost = self.vehicleList[t][1]
            for i in range(num_nodes):
                for j in range(num_nodes):
                    if i != j and self.x[t, i, j] is not None:
                        cost = self.my_round_int(veh_cost * self.locaList[i][j])
                        obj_expr.addTerms(-cost, self.x[t, i, j])

        self.model.setObjective(obj_expr, GRB.MAXIMIZE)

        # ==================== Structural Routing and Selection Constraints ====================
        
        # Eq. (2): Optional request assignment (each request r in R is served by at most one vehicle)
        for r in range(num_reqs):
            self.model.addConstr(gp.quicksum(self.y[r, t] for t in range(num_vehs)) <= 1, name=f"c2_r{r}")

        # Eqs. (3)-(6): Location visitation coupling, flow conservation, and single-visit degree bounds
        for t in range(num_vehs):
            is_used = self.model.addVar(vtype=GRB.BINARY, name=f"is_used_t{t}")
            for r in range(num_reqs):
                self.model.addConstr(is_used >= self.y[r, t])
            self.model.addConstr(self.x[t, self.DEPOT, self.DEPOT] == 1 - is_used)

            for i in range(num_nodes):
                if i != self.DEPOT:
                    reqs_pickup = [r for r in range(num_reqs) if self.requestList[r][2] == i]
                    reqs_delivery = [r for r in range(num_reqs) if self.requestList[r][3] == i]
                    all_reqs = reqs_pickup + reqs_delivery
                    
                    # Binary indicator for visiting customer location i in V \ {0}
                    v_active = self.model.addVar(vtype=GRB.BINARY, name=f"v_act_t{t}_i{i}")
                    
                    if all_reqs:
                        # Eqs. (3)-(4): Couple request assignment y^t_r with physical visits to f(r) and g(r)
                        self.model.addConstr(v_active <= gp.quicksum(self.y[r, t] for r in all_reqs))
                        for r in all_reqs:
                            self.model.addConstr(v_active >= self.y[r, t])
                    else:
                        self.model.addConstr(v_active == 0)
                    
                    in_flow = gp.quicksum(self.x[t, j, i] for j in range(num_nodes) if i != j and self.x[t, j, i] is not None)
                    out_flow = gp.quicksum(self.x[t, i, j] for j in range(num_nodes) if i != j and self.x[t, i, j] is not None)
                    
                    # Eqs. (5)-(6): Enforce flow conservation and at most one visit per customer location
                    self.model.addConstr(in_flow == v_active, name=f"c3_in_t{t}_i{i}")
                    self.model.addConstr(out_flow == v_active, name=f"c4_out_t{t}_i{i}")
                    self.model.addConstr(self.x[t, i, i] == 1 - v_active, name=f"c5_self_t{t}_i{i}")

            depot_in = gp.quicksum(self.x[t, j, self.DEPOT] for j in range(num_nodes) if self.DEPOT != j and self.x[t, j, self.DEPOT] is not None)
            depot_out = gp.quicksum(self.x[t, self.DEPOT, j] for j in range(num_nodes) if self.DEPOT != j and self.x[t, self.DEPOT, j] is not None)
            
            self.model.addConstr(depot_out == is_used, name=f"c6_depot_out_t{t}")
            self.model.addConstr(depot_in == is_used, name=f"c7_depot_in_t{t}")

        # Eqs. (7)-(8): Big-M linearization for subtour elimination and pickup-before-delivery precedence
        if self.bc_strategy in ['static', 'hybrid']:
            for t in range(num_vehs):
                # Eq. (7): MTZ subtour elimination u^t_d - u^t_o >= 1 - |V|(1 - x^t_{od})
                for i in range(n):
                    for j in range(n):
                        if i != j and self.x[t, i, j] is not None:
                            self.model.addConstr(self.u[t, i] - self.u[t, j] + n * self.x[t, i, j] <= n - 1)
                
                # Eq. (8): Pickup-before-delivery precedence u^t_{g(r)} - u^t_{f(r)} >= 1 - |V|(1 - y^t_r)
                for r in range(num_reqs):
                    p_node = self.requestList[r][2]
                    d_node = self.requestList[r][3]
                    if p_node != d_node:
                        self.model.addConstr(self.u[t, p_node] - self.u[t, d_node] + n * self.y[r, t] <= n - 1)
                        
        # Eq. (9): Bilateral big-M inequalities for continuous vehicle load tracking
        if self.bc_strategy == 'static':
            total_q = sum(self.requestList[r][1] for r in range(num_reqs))
            M = cap + min(cap, total_q)

            for t in range(num_vehs):
                self.model.addConstr(self.h[t, self.DEPOT] == 0)
                for i in range(num_nodes):
                    for j in range(num_nodes):
                        if i != j and self.x[t, i, j] is not None:
                            # Net load change Delta^t_d = sum_{r in f^{-1}(d)} q_r y^t_r - sum_{r in g^{-1}(d)} q_r y^t_r
                            reqs_pickup_j = [r for r in range(num_reqs) if self.requestList[r][2] == j]
                            reqs_delivery_j = [r for r in range(num_reqs) if self.requestList[r][3] == j]
                            
                            delta_tj = gp.quicksum(self.requestList[r][1] * self.y[r, t] for r in reqs_pickup_j) - \
                                       gp.quicksum(self.requestList[r][1] * self.y[r, t] for r in reqs_delivery_j)
                            
                            # Eq. (9): Delta^t_d - M(1 - x^t_{od}) <= h^t_d - h^t_o <= Delta^t_d + M(1 - x^t_{od})
                            self.model.addConstr(self.h[t, j] - self.h[t, i] >= delta_tj - M * (1 - self.x[t, i, j]))
                            self.model.addConstr(self.h[t, j] - self.h[t, i] <= delta_tj + M * (1 - self.x[t, i, j]))

        # Lexicographic Symmetry Breaking Constraint (SBC): y^t_r <= sum_{r' < r} y^{t-1}_{r'} (Section IV)
        for t in range(1, num_vehs):
            for r in range(num_reqs):
                self.model.addConstr(self.y[r, t] <= gp.quicksum(self.y[prev_r, t-1] for prev_r in range(r)))

        # ==================== Callback Metadata References ====================
        # Store variable handles and request parameters [q_r, f(r), g(r)] for lazy constraint separation
        self.model._x = {k: v for k, v in self.x.items() if v is not None}
        self.model._y = self.y
        self.model._DEPOT = self.DEPOT
        self.model._num_vehs = num_vehs
        self.model._num_reqs = num_reqs
        self.model._cap = cap
        self.model._cut_type = self.cut_type
        self.model._demand = {r: self.requestList[r][1] for r in range(num_reqs)}
        self.model._pickup = {r: self.requestList[r][2] for r in range(num_reqs)}
        self.model._dropoff = {r: self.requestList[r][3] for r in range(num_reqs)}


    # ==========================================
    # Callback 1: Hybrid Configurations (hybrid-path and hybrid-node, Section IV & Section VI-B1)
    # ==========================================
    @staticmethod
    def hybrid_benders_callback(model, where):
        if where == GRB.Callback.MIPSOL:
            x_vals = model.cbGetSolution(model._x)
            y_vals = model.cbGetSolution(model._y)

            for t in range(model._num_vehs):
                edges = [(i, j) for (veh, i, j), val in x_vals.items() if veh == t and val > 0.5]
                if not edges: continue

                adj = {u: v for u, v in edges}
                if model._DEPOT not in adj: continue
                
                # Reconstruct ordered candidate route R^t from the depot (Section IV-A)
                route_edges = []
                curr = model._DEPOT
                while curr in adj:
                    nxt = adj[curr]
                    route_edges.append((curr, nxt))
                    curr = nxt
                    if curr == model._DEPOT: break

                assigned_reqs = [r for r in range(model._num_reqs) if y_vals[r, t] > 0.5]
                load = 0
                onboard = set()
                
                # Simulate route R^t to track accumulated load Gamma and onboard set S_onboard (Algorithm 1, Lines 2-12)
                for k, (u, v) in enumerate(route_edges):
                    for r in assigned_reqs:
                        pk = model._pickup[r]
                        dp = model._dropoff[r]
                        if pk == dp and v == dp: pass
                        elif v == pk:
                            load += model._demand[r]
                            onboard.add(r)
                        elif v == dp and r in onboard:
                            load -= model._demand[r]
                            onboard.remove(r)

                    # Capacity violation Gamma > c: extract MCS S_conflict via greedy reduction (Algorithm 1, Lines 13-20)
                    if load > model._cap:
                        onboard_list = list(onboard)
                        onboard_list.sort(key=lambda r: model._demand[r], reverse=True)

                        conflict_core = []
                        core_load = 0
                        for r in onboard_list:
                            core_load += model._demand[r]
                            conflict_core.append(r)
                            if core_load > model._cap: break

                        if model._cut_type == 'node':
                            # Eq. (11): Node-based capacity cut broadcast across all t' in T (Section IV-C2 & IV-D)
                            prefix_nodes = {model._DEPOT}
                            for _, node in route_edges[:k+1]:
                                prefix_nodes.add(node)
                            rhs_bound = len(prefix_nodes) + len(conflict_core) - 2

                            for veh_id in range(model._num_vehs):
                                cut_expr = gp.quicksum(
                                    model._x[veh_id, src, tgt] 
                                    for src in prefix_nodes for tgt in prefix_nodes 
                                    if src != tgt and (veh_id, src, tgt) in model._x
                                )
                                cut_expr += gp.quicksum(model._y[r, veh_id] for r in conflict_core)
                                model.cbLazy(cut_expr <= rhs_bound)
                        else:
                            # Eq. (10): Path-based capacity cut broadcast across all t' in T (Section IV-C1 & IV-D)
                            path_edges = route_edges[:k+1]
                            rhs_bound = len(path_edges) + len(conflict_core) - 1
                            for veh_id in range(model._num_vehs):
                                cut_expr = gp.quicksum(
                                    model._x[veh_id, i, j] 
                                    for i, j in path_edges 
                                    if (veh_id, i, j) in model._x
                                )
                                cut_expr += gp.quicksum(model._y[r, veh_id] for r in conflict_core)
                                model.cbLazy(cut_expr <= rhs_bound)
                        
                        break

    # ==========================================
    # Callback 2: Full Configurations (full-path and full-node, Section VI-B1)
    # ==========================================
    @staticmethod
    def full_benders_callback(model, where):
        if where == GRB.Callback.MIPSOL:
            x_vals = model.cbGetSolution(model._x)
            y_vals = model.cbGetSolution(model._y)

            for t in range(model._num_vehs):
                edges = [(i, j) for (veh, i, j), val in x_vals.items() if veh == t and val > 0.5]
                if not edges: continue

                adj = {u: v for u, v in edges}

                # Stage 1: Separate Dantzig-Fulkerson-Johnson (DFJ) subtour elimination cuts
                visited = set()
                for u, v in edges:
                    if u not in visited:
                        curr = u
                        path = []
                        path_set = set()
                        while curr in adj and curr not in visited:
                            visited.add(curr)
                            path.append(curr)
                            path_set.add(curr)
                            curr = adj[curr]
                        
                        if curr in path_set:
                            cycle_idx = path.index(curr)
                            cycle = path[cycle_idx:]
                            if model._DEPOT not in cycle:
                                S = set(cycle)
                                for veh_id in range(model._num_vehs):
                                    cut_expr = gp.quicksum(
                                        model._x[veh_id, i, j] 
                                        for i in S for j in S 
                                        if i != j and (veh_id, i, j) in model._x
                                    )
                                    model.cbLazy(cut_expr <= len(S) - 1)

                if model._DEPOT not in adj: continue
                
                route_edges = []
                curr = model._DEPOT
                while curr in adj:
                    nxt = adj[curr]
                    route_edges.append((curr, nxt))
                    curr = nxt
                    if curr == model._DEPOT: break

                assigned_reqs = [r for r in range(model._num_reqs) if y_vals[r, t] > 0.5]
                
                # Stage 2: Separate path-based pickup-before-delivery precedence cuts
                seen_nodes = {model._DEPOT}
                for k, (u, v) in enumerate(route_edges):
                    seen_nodes.add(v)
                    for r in assigned_reqs:
                        pk = model._pickup[r]
                        dp = model._dropoff[r]
                        if pk != dp and v == dp and pk not in seen_nodes:
                            path_edges = route_edges[:k+1]
                            for veh_id in range(model._num_vehs):
                                cut_expr = gp.quicksum(
                                    model._x[veh_id, i, j] 
                                    for i, j in path_edges 
                                    if (veh_id, i, j) in model._x
                                )
                                model.cbLazy(cut_expr + model._y[r, veh_id] <= len(path_edges))

                # Stage 3: Separate capacity cuts via Algorithm 1 (Eq. (11) node-based or Eq. (10) path-based)
                load = 0
                onboard = set()
                for k, (u, v) in enumerate(route_edges):
                    for r in assigned_reqs:
                        pk = model._pickup[r]
                        dp = model._dropoff[r]
                        if pk == dp and v == dp: pass
                        elif v == pk:
                            load += model._demand[r]
                            onboard.add(r)
                        elif v == dp and r in onboard:
                            load -= model._demand[r]
                            onboard.remove(r)

                    if load > model._cap:
                        onboard_list = list(onboard)
                        onboard_list.sort(key=lambda r: model._demand[r], reverse=True)
                        
                        conflict_core = []
                        core_load = 0
                        for r in onboard_list:
                            core_load += model._demand[r]
                            conflict_core.append(r)
                            if core_load > model._cap: break

                        if model._cut_type == 'node':
                            prefix_nodes = {model._DEPOT}
                            for _, node in route_edges[:k+1]:
                                prefix_nodes.add(node)
                            rhs_bound = len(prefix_nodes) + len(conflict_core) - 2

                            for veh_id in range(model._num_vehs):
                                cut_expr = gp.quicksum(
                                    model._x[veh_id, src, tgt] 
                                    for src in prefix_nodes for tgt in prefix_nodes 
                                    if src != tgt and (veh_id, src, tgt) in model._x
                                )
                                cut_expr += gp.quicksum(model._y[r, veh_id] for r in conflict_core)
                                model.cbLazy(cut_expr <= rhs_bound)
                        else:
                            path_edges = route_edges[:k+1]
                            rhs_bound = len(path_edges) + len(conflict_core) - 1
                            for veh_id in range(model._num_vehs):
                                cut_expr = gp.quicksum(
                                    model._x[veh_id, i, j] 
                                    for i, j in path_edges 
                                    if (veh_id, i, j) in model._x
                                )
                                cut_expr += gp.quicksum(model._y[r, veh_id] for r in conflict_core)
                                model.cbLazy(cut_expr <= rhs_bound)
                        
                        break

    def solve(self, time_limit=3600, assumption_file=None):
        self.genMipModel()
        
        if time_limit is not None:
            print(f"[Gurobi {self.bc_strategy}_bc] Setting time limit to {time_limit} seconds")
            self.model.setParam('TimeLimit', time_limit)
            
        self.model.setParam('Threads', 1) 
        
        # Enable lazy constraint callbacks for branch-and-cut execution
        self.model.setParam('LazyConstraints', 1)

        log_file = f"{self.insName}.out"
        with open(log_file, "w") as f:
            def log(msg):
                print(msg)
                f.write(msg + "\n")
                f.flush()
                
            start_time = time.time()
            
            self.best_incumbent_obj = float('-inf')

            def combined_callback(model, where):
                # 1. Record validated incumbent objective z, dual upper bound, and elapsed runtime T_best (Section VI-C)
                if where == gp.GRB.Callback.MIP:
                    obj_bst = model.cbGet(gp.GRB.Callback.MIP_OBJBST)
                    bnd = model.cbGet(gp.GRB.Callback.MIP_OBJBND)
                    runtime = model.cbGet(gp.GRB.Callback.RUNTIME)
                    
                    if obj_bst > self.best_incumbent_obj and obj_bst > -1e99:
                        self.best_incumbent_obj = obj_bst
                        
                        gap_str = "N/A"
                        if abs(obj_bst) > 1e-5: 
                            gap = abs(bnd - obj_bst) / abs(obj_bst) * 100.0
                            gap_str = f"{gap:.2f}%"
                            
                        with open(log_file, "a") as cb_f:
                            cb_f.write(f"[Incumbent] Time: {runtime:.2f}s | Obj: {obj_bst:.1f} | Bound: {bnd:.1f} | Gap: {gap_str}\n")
                                
                # 2. Invoke lazy constraint separation callbacks according to the active MIP configuration
                if self.bc_strategy == 'hybrid':
                    self.hybrid_benders_callback(model, where)
                elif self.bc_strategy == 'full':
                    self.full_benders_callback(model, where)

            self.model.optimize(combined_callback)
            
            elapsed = time.time() - start_time

            if self.model.Status == gp.GRB.OPTIMAL or self.model.Status == gp.GRB.TIME_LIMIT:
                if self.model.SolCount > 0:
                    raw_model = [v.VarName for v in self.model.getVars() if v.X > 0.5 and (v.VarName.startswith('x') or v.VarName.startswith('y'))]
                    filtered_model = PPDSP_utils.convert_model(raw_model)

                    log(f"[Gurobi {self.bc_strategy}_{self.cut_type}] Status: {self.model.Status}")
                    log(f"[Gurobi {self.bc_strategy}_{self.cut_type}] BEST OBJ: {self.model.ObjVal}")
                    log(f"[Gurobi {self.bc_strategy}_{self.cut_type}] BEST BOUND: {self.model.ObjBound}")
                    log(f"[Gurobi {self.bc_strategy}_{self.cut_type}] Runtime: {elapsed:.3f} sec")
                    
                    PPDSP_utils.printVehRoutes(self, filtered_model, log_file)
                    PPDSP_utils.evaluateSolution(self, filtered_model, log_file)
                    return filtered_model
                else:
                    log(f"[Gurobi {self.bc_strategy}_{self.cut_type}] Reached Time Limit, NO feasible solution.")
                    try:
                        best_bound = self.model.ObjBound
                        log(f"Best objective -, best bound {best_bound}")
                    except:
                        pass
            else:
                log(f"[Gurobi {self.bc_strategy}_{self.cut_type}] Failed to find solution. Status: {self.model.Status}")
            return None
