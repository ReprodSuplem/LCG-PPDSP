# ppdsp_maxsat.py
# Weighted partial MaxSAT propositional encoding and topological order encoding
# (Section V-A, Section V-B, Section V-C, Section VI-B3).

import os
from ppdsp_ins_gen import PPDSP_reform
from ppdsp_utils import PPDSP_utils
from pysat.pb import *
from pysat.formula import *

class PPDSP_MaxSAT(PPDSP_reform):
    """
    Weighted partial MaxSAT formulation Phi = (C_hard, C_soft) supporting structural
    Lazy Clause Generation configurations MaxSAT-path (Eq. (20)) and MaxSAT-order (Eq. (21))
    (Section V, Section VI-B3).
    """
    def __init__(self, pdpName, num_reqs, num_vehs, capacity, knn, increment=None, cut_type='path'):
        super().__init__(pdpName, num_reqs, num_vehs, capacity, knn, increment)
        self.knn = int(knn)
        self.cut_type = cut_type.lower()
        self.wcnf = WCNF()
        self.cnf = CNF()
        self.vpool = None
        
        self.insName = f"ppdsp_{pdpName}_r{num_reqs}v{num_vehs}k{knn}_maxsat"

    def atLeastOne(self, varList):
        self.wcnf.append(varList)

    def atMostOne(self, varList):
        # Pairwise At-Most-One (AMO) encoding (Section V-B, Eqs. (12)-(13))
        for i in range(len(varList)):
            for j in range(1+i, len(varList)):
                self.wcnf.append([(-1 * varList[i]), (-1 * varList[j])])

    def exactlyOne(self, varList):
        self.atMostOne(varList)
        self.atLeastOne(varList)

    def twoSumsEqvt(self, litList1, litList2):
        return (
            [[-x] + litList2 for x in litList1] +
            [[-y] + litList1 for y in litList2]
        )

    # Section V-A: Weighted soft clauses C_soft mapping Eq. (1) into penalty minimization
    def genSoftClause(self):
        # 1. Arc traversal cost soft clauses: ((~x^t_{ij}), l_{ij}) for all (i, j) in E, t in T
        for i in range(self.lenOfVehicle):
            for j in range(1+self.lenOfLocation):
                for k in range(1+self.lenOfLocation):
                    if k != j:
                        cost = self.my_round_int(self.vehicleList[i][1] * self.locaList[j][k])
                        if cost > 0:
                            self.wcnf.append([-self.xVarList[i][j][k]], weight=cost)

        # 2. Uncollected request revenue soft clauses: ((bigvee_{t in T} y^t_r), p_r) for all r in R
        for r in range(self.lenOfRequest):
            profit = self.requestList[r][0]
            varList = []
            for t in range(self.lenOfVehicle):
                varList.append(self.yVarList[r][t])
            # Falsifying this clause indicates request r is rejected by all vehicles, incurring penalty p_r
            self.wcnf.append(varList, weight=profit)

    # Eq. (12): Pairwise AMO encoding for optional request assignment in Eq. (2)
    def genHardClauseForSelection(self):
        for i in range(self.lenOfRequest):
            varList = []
            for j in range(self.lenOfVehicle):
                varList.append(self.yVarList[i][j])
            # Enforce at-most-one vehicle assignment per request r in R
            self.atMostOne(varList)

    # Eq. (14): Logical implication coupling request selection y^t_r with pickup visitation f(r)
    def genHardClauseForPickupLinkage(self):
        for i in range(self.lenOfRequest):
            pickup = self.requestList[i][2]
            dropoff = self.requestList[i][3]
            for j in range(self.lenOfVehicle):
                varList = [-self.yVarList[i][j]]
                for k in range(1+self.lenOfLocation):
                    if k != pickup and k != dropoff:
                        if self.adjMatrx[k][pickup] != 0:
                            varList.append(self.xVarList[j][k][pickup])
                self.wcnf.append(varList)

    # Eq. (15): Logical implication coupling request selection y^t_r with delivery visitation g(r)
    def genHardClauseForDropoffLinkage(self):
        for i in range(self.lenOfRequest):
            dropoff = self.requestList[i][3]
            for j in range(self.lenOfVehicle):
                varList = [-self.yVarList[i][j]]
                for k in range(self.lenOfLocation):
                    if k != dropoff:
                        if self.adjMatrx[k][dropoff] != 0:
                            varList.append(self.xVarList[j][k][dropoff])
                self.wcnf.append(varList)

    # Eq. (16): Flow conservation clauses ensuring path continuity across customer locations
    def genHardClauseForFlowBalance(self):
        for i in range(self.lenOfVehicle):
            for j in range(1+self.lenOfLocation):
                litList1 = []
                litList2 = []
                for k in range(1+self.lenOfLocation):
                    if k != j and self.adjMatrx[j][k] != 0:
                        litList1.append(self.xVarList[i][j][k])
                    if k != j and self.adjMatrx[k][j] != 0:
                        litList2.append(self.xVarList[i][k][j])
                cnf_obj = self.twoSumsEqvt(litList1, litList2)
                for clause in cnf_obj:
                    self.wcnf.append(clause)

    # Eq. (13): Pairwise AMO encoding for node out-degree bound in Eq. (6)
    def genHardClauseForDegreeConstraint(self):
        for i in range(self.lenOfVehicle):
            for j in range(1+self.lenOfLocation):
                varList = []
                for k in range(1+self.lenOfLocation):
                    if k != j:
                        varList.append(self.xVarList[i][j][k])
                self.atMostOne(varList)

    # Eq. (17): Domain monotonicity clauses nu^t_{d,eps} => nu^t_{d,eps+1} for order literals (Section V-C)
    def genHardClauseForDomainTransitive(self):
        num_bits = self.lenOfLocation - 1 
        for t in range(self.lenOfVehicle):
            for i in range(self.lenOfLocation):
                for p in range(num_bits - 1):
                    self.wcnf.append([-self.nuVarList[t][i][p], self.nuVarList[t][i][p+1]])

    # Eq. (18): Subtour elimination via step-wise position advancement and boundary clauses (Section V-C)
    def genHardClauseForEq7(self): 
        num_bits = self.lenOfLocation - 1 
        last_bit_idx = num_bits - 1 
        for t in range(self.lenOfVehicle):
            for j in range(self.lenOfLocation):
                for k in range(self.lenOfLocation):
                    if j == k: continue
                    if self.adjMatrx[j][k] == 0: continue 
                    for p in range(num_bits):
                        clause = [-self.xVarList[t][j][k], -self.nuVarList[t][k][p]]
                        if p > 0:
                            clause.append(self.nuVarList[t][j][p-1])
                        self.wcnf.append(clause)
                    
                    clause_boundary = [-self.xVarList[t][j][k], self.nuVarList[t][j][last_bit_idx]]
                    self.wcnf.append(clause_boundary)

    # Eq. (19): Pickup-before-delivery precedence via order literals and boundary clauses (Section V-C)
    def genHardClauseForEq8(self): 
        num_bits = self.lenOfLocation - 1 
        last_bit_idx = num_bits - 1 
        for i in range(self.lenOfRequest):
            pickup = self.requestList[i][2]
            dropoff = self.requestList[i][3]
            
            if pickup == dropoff:
                continue
                
            for t in range(self.lenOfVehicle):
                for p in range(num_bits):
                    clause = [-self.yVarList[i][t], -self.nuVarList[t][dropoff][p]]
                    if p > 0:
                        clause.append(self.nuVarList[t][pickup][p-1])
                    self.wcnf.append(clause)
                
                clause_boundary = [-self.yVarList[i][t], self.nuVarList[t][pickup][last_bit_idx]]
                self.wcnf.append(clause_boundary)

    # Enforce sparse graph topology G = (V, E) by fixing non-existent arcs (i, j) not in E to false
    def genHardClauseForKnn(self): 
        for t in range(self.lenOfVehicle):
            for i in range(len(self.adjMatrx)):
                for j in range(len(self.adjMatrx[i])):
                    if self.adjMatrx[i][j] == 0:
                        x_var = self.xVarList[t][i][j]
                        self.wcnf.append([-x_var])

    # Node visitation pruning for unrequested locations in complete graphs (k-NN = 0)
    def genHardClauseFoRec(self):
        node_requests = [[] for _ in range(self.lenOfLocation)]
        for r in range(self.lenOfRequest):
            pickup = self.requestList[r][2]
            dropoff = self.requestList[r][3]
            node_requests[pickup].append(r)
            if dropoff != pickup:
                node_requests[dropoff].append(r)
            
        for k in range(self.lenOfVehicle):
            for i in range(self.lenOfLocation):
                service_lits = [self.yVarList[r][k] for r in node_requests[i]]
                for j in range(self.lenOfLocation + 1): 
                    if j == i: continue
                    x_var = self.xVarList[k][j][i]
                    self.wcnf.append([-x_var] + service_lits)

    # Propositional Symmetry Breaking Constraint (SBC): y^t_r => bigvee_{r' < r} y^{t-1}_{r'} (Section V-D)
    def genHardClauseForSbc(self):
        groups = PPDSP_utils.get_sbc_groups(self.vehicleList)
        for key, veh_ids in groups.items():
            for i in range(len(veh_ids) - 1):
                leader = veh_ids[i]
                follower = veh_ids[i+1]
                for r in range(self.lenOfRequest):
                    clause = [-self.yVarList[r][follower]]
                    for prev_r in range(r):
                        clause.append(self.yVarList[prev_r][leader])
                    self.wcnf.append(clause)

    # Active vehicle arc pruning: x^t_{ij} => bigvee_{r in R} y^t_r
    def genHardClauseForEVP(self):
        for t in range(self.lenOfVehicle):
            any_req_lits = [self.yVarList[r][t] for r in range(self.lenOfRequest)]
            for i in range(1+self.lenOfLocation):
                for j in range(1+self.lenOfLocation):
                    if i != j and self.adjMatrx[i][j] != 0:
                        x_var = self.xVarList[t][i][j]
                        self.wcnf.append([-x_var] + any_req_lits)

    def genMaxsatFormular(self):
        self.genXVarList()
        self.genYVarList()
        self.genNuVarList()

        self.genSoftClause()
        self.genHardClauseForSelection()
        self.genHardClauseForPickupLinkage()
        self.genHardClauseForDropoffLinkage()
        self.genHardClauseForFlowBalance()
        self.genHardClauseForDegreeConstraint()
        
        self.genHardClauseForDomainTransitive()
        self.genHardClauseForEq7()
        self.genHardClauseForEq8()
        
        self.genHardClauseFoRec() if self.knn == 0 else self.genHardClauseForKnn()
        self.genHardClauseForSbc()
        self.genHardClauseForEVP()

        print(f"[MaxSAT] Generating instance: {self.insName}.wcnf ...")
        self.wcnf.extend(self.cnf)
        self.wcnf.to_file(self.insName + ".wcnf")
        PPDSP_utils.export_meta(self, self.insName + ".meta")

    def solve(self, verbose=1, time_limit=3600, assumption_file=None):
        """
        Execute UWrMaxSat with the embedded CEGAR structural LCG loop (Algorithm 1) and
        disabled internal SCIP hybrid solver (-no-scip) (Section V-D, Section VI-B3).
        """
        wcnf_file = self.insName + ".wcnf"
        lastY = self.getLastYVarID()
        meta_file = self.insName + ".meta"
        log_file  = wcnf_file + "_" + self.cut_type + ".out"

        cmd = f"stdbuf -oL uwrmaxsat -no-bin -no-sat -no-par -no-scip -ppdsp-cut-type={self.cut_type} -ppdsp-time={time_limit} -ppdsp-lastY={lastY} -ppdsp={meta_file}"
        if assumption_file is not None and os.path.exists(assumption_file):
            cmd += f" -ppdsp-assume={assumption_file}"
            print(f"  [Info] Injected assumption file: {assumption_file}")
            
        cmd += f" {wcnf_file} | tee {log_file}"
        print(f"[UWrMaxSAT] Executing solver command:\n  {cmd}")
        os.system(cmd)

        model = []
        with open(log_file, "r") as f:
            for line in f:
                line = line.strip()
                if line.startswith("v "):
                    for lit in line.split()[1:]: 
                        if lit != "0":
                            model.append(int(lit))

        if not model:
            with open(log_file, "a") as f:
                f.write("\n[UWrMaxSAT] No feasible solution found.\n")
            print("[UWrMaxSAT] No feasible solution found.")
            return None

        filtered_model = PPDSP_utils.extractXYModel(self, model)

        PPDSP_utils.printVehRoutes(self, filtered_model, log_file)
        obj_val = PPDSP_utils.evaluateSolution(self, filtered_model, log_file)

        with open(log_file, "a") as f:
            f.write(f"[UWrMaxSAT] OBJ: {obj_val}\n")

        return filtered_model
