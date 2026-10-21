# ppdsp_cpsat.py
# Constraint Programming with Lazy Clause Generation (CP-SAT) formulation (Section III-B, Section VI-B2).

from ppdsp_ins_gen import PPDSP_reform
from ppdsp_utils import PPDSP_utils
import time
from ortools.sat.python import cp_model

class SolutionLogger(cp_model.CpSolverSolutionCallback):
    """
    Solution callback recording incumbent objective values z, dual bounds, and
    discovery timestamps T_best during the CP-SAT search (Section VI-C).
    """
    def __init__(self, log_func):
        cp_model.CpSolverSolutionCallback.__init__(self)
        self.log_func = log_func
        self.solution_count = 0

    def OnSolutionCallback(self):
        self.solution_count += 1
        obj = self.ObjectiveValue()
        bound = self.BestObjectiveBound()
        time_elapsed = self.WallTime()

        if obj == 0:
            gap = float('inf') if bound != 0 else 0.0
        else:
            gap = abs(bound - obj) / abs(obj) * 100.0

        msg = f"[Incumbent {self.solution_count:02d}] Time: {time_elapsed:6.2f}s | Obj: {obj:6.0f} | Bound: {bound:6.0f} | Gap: {gap:6.2f}%"
        self.log_func(msg)

class PPDSP_CPSAT(PPDSP_reform):
    """
    Google OR-Tools CP-SAT model employing the global circuit propagator for flow conservation
    and subtour elimination, coupled with conditional integer constraints for visit ordering
    and vehicle load accumulation (Section VI-B2).
    """
    def __init__(self, pdpName, num_reqs, num_vehs, capacity, knn, increment=None):
        super().__init__(pdpName, num_reqs, num_vehs, capacity, knn, increment)
        self.knn = int(knn)
        
        self.model = cp_model.CpModel()
        
        # Configure instance identifier for the CP-SAT configuration (Section VI-B2)
        self.insName = f"ppdsp_{pdpName}_r{num_reqs}v{num_vehs}k{knn}_cpsat"
        
        # Decision variable dictionaries for the CP-SAT formulation
        self.x = {}      
        self.y = {}      
        self.pos = {}    
        self.load = {}   
        self.delta = {}  
        self.active = {} 

    def genCpModel(self):
        # Allocate routing variables x^t_{od} and assignment variables y^t_r (Section III-A)
        self.genXVarList()
        self.genYVarList()

        # Graph dimension and depot index definitions (Section III-A)
        self.DEPOT = self.lenOfLocation
        num_nodes = 1 + self.lenOfLocation
        num_reqs = self.lenOfRequest
        num_vehs = self.lenOfVehicle
        n = self.lenOfLocation

        # ==================== Decision Variable Declarations (Section III-A) ====================
        for t in range(num_vehs):
            cap = int(self.vehicleList[t][0])
            for r in range(num_reqs):
                # Binary request assignment variables y^t_r in {0, 1}
                var_id = self.yVarList[r][t]
                self.y[r, t] = self.model.NewBoolVar(f'y{var_id}')

            for i in range(num_nodes):
                self.active[t, i] = self.model.NewBoolVar(f'act_t{t}_i{i}')
                
                if i != self.DEPOT:
                    # Integer visit sequence variables 0 <= u^t_v <= |V|-2
                    self.pos[t, i] = self.model.NewIntVar(0, n - 1, f'pos_t{t}_i{i}')
                    
                # Integer load variables 0 <= h^t_v <= c and net load change variables Delta^t_d
                self.load[t, i] = self.model.NewIntVar(0, cap, f'load_t{t}_i{i}')
                self.delta[t, i] = self.model.NewIntVar(-cap, cap, f'delta_t{t}_i{i}')

                # Binary routing variables x^t_{od} in {0, 1}
                for j in range(num_nodes):
                    if i == j: 
                        var_id = self.xVarList[t][i][i]
                        self.x[t, i, i] = self.model.NewBoolVar(f'x{var_id}')
                    elif self.adjMatrx[i][j] != 0:
                        var_id = self.xVarList[t][i][j]
                        self.x[t, i, j] = self.model.NewBoolVar(f'x{var_id}')

        # ==================== Structural Routing and Selection Constraints ====================
        
        # Eq. (2): Optional request assignment (At-Most-One across fleet T)
        for r in range(num_reqs):
            self.model.AddAtMostOne([self.y[r, t] for t in range(num_vehs)])

        # Eqs. (3)-(4) & Eq. (9): Location visitation coupling and net load change Delta^t_d
        for t in range(num_vehs):
            self.model.Add(self.load[t, self.DEPOT] == 0)  
            self.model.Add(self.delta[t, self.DEPOT] == 0)

            is_used = self.model.NewBoolVar(f'is_used_t{t}')
            self.model.AddMaxEquality(is_used, [self.y[r, t] for r in range(num_reqs)])
            self.model.Add(self.x[t, self.DEPOT, self.DEPOT] == is_used.Not()) 

            for i in range(num_nodes):
                if i != self.DEPOT:
                    self.model.Add(self.x[t, i, i] == self.active[t, i].Not())
                    
                    # Sets f^{-1}(i) and g^{-1}(i) of requests originating and terminating at node i (Section III-B)
                    reqs_pickup = [r for r in range(num_reqs) if self.requestList[r][2] == i]
                    reqs_delivery = [r for r in range(num_reqs) if self.requestList[r][3] == i]
                    all_reqs = reqs_pickup + reqs_delivery

                    for r in all_reqs:
                        self.model.AddImplication(self.y[r, t], self.active[t, i])
                        
                    if self.knn == 0:
                        if all_reqs:
                            self.model.AddMaxEquality(self.active[t, i], [self.y[r, t] for r in all_reqs])
                        else:
                            self.model.Add(self.active[t, i] == 0)
                            
                    self.model.AddImplication(is_used.Not(), self.active[t, i].Not())

                    pickup_sum = sum(self.requestList[r][1] * self.y[r, t] for r in reqs_pickup)
                    delivery_sum = sum(self.requestList[r][1] * self.y[r, t] for r in reqs_delivery)
                    self.model.Add(self.delta[t, i] == pickup_sum - delivery_sum)

        # Eqs. (5)-(7) & Eq. (9): Global circuit propagator, visit ordering, and load accumulation (Section VI-B2)
        for t in range(num_vehs):
            arcs = []
            for i in range(num_nodes):
                if (t, i, i) in self.x:
                    arcs.append((i, i, self.x[t, i, i]))
                for j in range(num_nodes):
                    if i != j and (t, i, j) in self.x:
                        arcs.append((i, j, self.x[t, i, j]))
                        if j != self.DEPOT:
                            if i != self.DEPOT:
                                self.model.Add(self.pos[t, j] == self.pos[t, i] + 1).OnlyEnforceIf(self.x[t, i, j])
                            self.model.Add(self.load[t, j] == self.load[t, i] + self.delta[t, j]).OnlyEnforceIf(self.x[t, i, j])
            self.model.AddCircuit(arcs)

            # Eq. (8): Pickup-before-delivery precedence constraint
            for r in range(num_reqs):
                p_node = self.requestList[r][2]
                d_node = self.requestList[r][3]
                if p_node != d_node:
                    self.model.Add(self.pos[t, p_node] < self.pos[t, d_node]).OnlyEnforceIf(self.y[r, t])

        # ==================== Symmetry Breaking and Active Vehicle Coupling ====================
        
        # Lexicographic Symmetry Breaking Constraint (SBC): y^t_r <= sum_{r' < r} y^{t-1}_{r'} (Section IV)
        for t in range(1, num_vehs):
            for r in range(num_reqs):
                self.model.Add(self.y[r, t] <= sum(self.y[prev_r, t-1] for prev_r in range(r)))

        # Active vehicle arc pruning
        for t in range(num_vehs):
            is_used = self.model.NewBoolVar(f'evp_is_used_t{t}')
            self.model.AddMaxEquality(is_used, [self.y[r, t] for r in range(num_reqs)])
            for i in range(num_nodes):
                for j in range(num_nodes):
                    if i != j and (t, i, j) in self.x:
                        self.model.AddImplication(is_used.Not(), self.x[t, i, j].Not())

        # ==================== Objective Function: Eq. (1) ====================
        
        # Maximize net operational profit: sum_{t in T} sum_{r in R} p_r y^t_r - sum_{t in T} sum_{(o,d) in E} l_{od} x^t_{od}
        obj_terms = []
        
        # 1. Total revenue collected from served requests
        for r in range(num_reqs):
            profit = self.requestList[r][0]
            for t in range(num_vehs):
                obj_terms.append(profit * self.y[r, t])
                
        # 2. Total fleet routing cost
        for t in range(num_vehs):
            veh_cost = self.vehicleList[t][1]
            for i in range(num_nodes):
                for j in range(num_nodes):
                    if i != j and (t, i, j) in self.x:
                        cost = self.my_round_int(veh_cost * self.locaList[i][j])
                        obj_terms.append(-cost * self.x[t, i, j])

        self.model.Maximize(sum(obj_terms))

    def solve(self, time_limit=3600, assumption_file=None):
        self.genCpModel()
        solver = cp_model.CpSolver()
        
        if time_limit is not None:
            solver.parameters.max_time_in_seconds = time_limit
        
        solver.parameters.num_search_workers = 1
        solver.parameters.log_search_progress = True 
        
        print(f"[CP-SAT] Setting time limit to {time_limit} seconds with {solver.parameters.num_search_workers} workers.")
        
        log_file = f"{self.insName}.out"
        def log(msg):
            print(msg)
            with open(log_file, "a") as f:
                f.write(msg + "\n")
                
        with open(log_file, "w") as f:
            f.write(f"--- Starting CP-SAT for {self.insName} ---\n")

        solution_logger = SolutionLogger(log)

        start_time = time.time()
        status = solver.Solve(self.model, solution_logger)
        elapsed = time.time() - start_time

        status_name = solver.StatusName(status)
        log(f"[CP-SAT] Status: {status_name}")
        
        if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
            raw_model = []
            
            for t in range(self.lenOfVehicle):
                for i in range(1 + self.lenOfLocation):
                    for j in range(1 + self.lenOfLocation):
                        if i != j and (t, i, j) in self.x:
                            if solver.Value(self.x[t, i, j]):
                                var_id = self.xVarList[t][i][j]
                                raw_model.append(f"x{var_id}")
                                
            for r in range(self.lenOfRequest):
                for t in range(self.lenOfVehicle):
                    if (r, t) in self.y:
                        if solver.Value(self.y[r, t]):
                            var_id = self.yVarList[r][t]
                            raw_model.append(f"y{var_id}")

            filtered_model = PPDSP_utils.convert_model(raw_model)

            log(f"[CP-SAT] FINAL OBJ: {solver.ObjectiveValue()}")
            log(f"[CP-SAT] Total Runtime: {elapsed:.3f} sec")
            
            PPDSP_utils.printVehRoutes(self, filtered_model, log_file)
            PPDSP_utils.evaluateSolution(self, filtered_model, log_file)
            
            log("===== RAW XY MODEL =====")
            log(" ".join(str(v) for v in filtered_model))
            
            return filtered_model
        else:
            log("[CP-SAT] No feasible solution found.")
            log(f"[CP-SAT] Total Runtime: {elapsed:.3f} sec")
            return None
