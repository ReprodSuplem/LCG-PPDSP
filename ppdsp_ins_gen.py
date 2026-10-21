# ppdsp_ins_gen.py
# Location-based network representation and decision variable indexing (Section III-A, Section V).

import os
import sys
import math
import pandas as pd
from pysat.formula import IDPool

class PPDSP_reform:
    """
    Base class for location-based PPDSP graph data and variable index management (Section III-A).
    Maintains decision variable mappings for routing arcs x^t_{od}, request assignments y^t_r,
    topological order literals nu^t_{d,eps}, visit positions u^t_v, and vehicle loads h^t_v.
    """
    def __init__(self, pdpName: str, num_reqs: int, num_vehs: int, capacity: int, knn: int = 3, increment=None):
        self.registry = increment
        self.varID = 0
        self.vpool = None
        self.id2Var = None

        self.pdpName = pdpName
        self.num_reqs = int(num_reqs)
        self.num_vehs = int(num_vehs)
        self.capacity = int(capacity)
        self.knn = int(knn)

        self.adjMatrx = []
        self.coordinates = []
        self.locaList = []
        self.requestList = []
        self.vehicleList = []
        
        self.lenOfLocation = 0
        self.lenOfRequest = 0
        self.lenOfVehicle = 0
        
        self.readCSV()
        
        # Customer locations V \ {0} are indexed 0 .. lenOfLocation-1; depot is indexed at lenOfLocation
        self.lenOfLocation = len(self.locaList) - 1 
        self.lenOfRequest = len(self.requestList)
        self.lenOfVehicle = len(self.vehicleList)
        
        # Decision variable index tables (Section III-A and Section V-C)
        self.xVarList = [[[0] * (1 + self.lenOfLocation) for _ in range(1 + self.lenOfLocation)] for _ in range(self.lenOfVehicle)]
        self.yVarList = [[0] * self.lenOfVehicle for _ in range(self.lenOfRequest)]
        
        self.nuVarList = [[[0] * self.lenOfLocation for _ in range(1 + self.lenOfLocation)] for _ in range(self.lenOfVehicle)]
        self.uVarList = [[0] * (1 + self.lenOfLocation) for _ in range(self.lenOfVehicle)]
        self.hVarList = [[0] * (1 + self.lenOfLocation) for _ in range(self.lenOfVehicle)]
        
        self.genVariables()

    def my_round_int(self, x: float) -> int:
        return int((x * 2 + 1) // 2)

    def readCSV(self):
        """
        Load synthesized instance files (fleet T, coordinates V, sparse adjacency matrix E,
        and request pool R) and compute all-pairs shortest-path travel costs l_{od} (Section VI-A).
        """
        veh_file = f"vehicleInfo{self.num_vehs}_{self.pdpName}.csv"
        if not os.path.exists(veh_file):
            print(f"[Error] Vehicle file not found: {veh_file}")
            sys.exit(1)
            
        veh_df = pd.read_csv(veh_file)
        for _, row in veh_df.iterrows():
            cap = self.my_round_int(float(row.iloc[1]))
            cost_factor = float(row.iloc[2])
            self.vehicleList.append([cap, cost_factor])
                
        node_file = f"2DNode_{self.pdpName}.csv"
        self.coordinates = pd.read_csv(node_file, header=None).values.tolist()
        self.lenOfCoord = len(self.coordinates)

        adj_file = f"adjMatrx{self.knn}_{self.pdpName}.csv"
        adj_df = pd.read_csv(adj_file, header=None)
        for _, row in adj_df.iterrows():
            self.adjMatrx.append([self.my_round_int(float(val)) for val in row])

        for i in range(self.lenOfCoord):
            tmpList = []
            for j in range(self.lenOfCoord):
                if self.adjMatrx[i][j] == 0: # Non-adjacent arc (o, d) not in E
                    tmpList.append(999999)
                elif self.adjMatrx[i][j] == 2: # Zero-cost arc
                    tmpList.append(0)
                elif self.adjMatrx[i][j] == 1: # Directed edge (o, d) in E
                    tmpList.append(
                        self.my_round_int(
                            math.dist((self.coordinates[i][0], self.coordinates[i][1]),
                                      (self.coordinates[j][0], self.coordinates[j][1]))))
            self.locaList.append(tmpList)

        # Compute all-pairs shortest-path distances via the Floyd-Warshall algorithm (Section VI-A)
        self.floyd(self.locaList)
        self.lenOfLocation = self.lenOfCoord - 1

        # Load request parameters [p_r, q_r, f(r), g(r)] for all r in R (Section III-A)
        req_file = f"requestInfo{self.num_reqs}_{self.pdpName}.csv"
        req_df = pd.read_csv(req_file, header=None)
        for _, row in req_df.iterrows():
            profit = self.my_round_int(float(row.iloc[0]))
            size = self.my_round_int(float(row.iloc[1]))
            pk = self.my_round_int(float(row.iloc[2]))
            dp = self.my_round_int(float(row.iloc[3]))
            self.requestList.append([profit, size, pk, dp])

    def floyd(self, tmpMatrix):
        """
        Floyd-Warshall algorithm ensuring arc travel costs l_{od} satisfy the triangle inequality (Section VI-A).
        """
        for i in range(len(tmpMatrix)):
            for j in range(len(tmpMatrix)):
                for k in range(len(tmpMatrix)):
                    tmpMatrix[j][k] = min(tmpMatrix[j][k], tmpMatrix[j][i] + tmpMatrix[i][k])

    def newVarID(self, var_type=None, *args):
        if self.registry is not None:
            if var_type is not None:
                return self.registry.get_id(var_type, *args)
            else:
                if self.vpool is None:
                    self.vpool = IDPool(start_from=self.get_vpool_start_id())
                return self.vpool.id()
        else:
            self.varID += 1
            return self.varID

    # ========================== Decision Variable Index Allocation ==========================
    
    def genVariables(self):
        self.genXVarList()
        self.genYVarList()
        self.genNuVarList()
        self.genUVarList()
        self.genHVarList()

    def genXVarList(self):
        # Binary routing variables x^t_{od} in {0, 1} (Section III-A)
        for i in range(len(self.xVarList)):
            for j in range(len(self.xVarList[i])):
                for k in range(len(self.xVarList[i][j])):
                    self.xVarList[i][j][k] = self.newVarID('x', i, j, k)

    def genYVarList(self):
        # Binary request assignment variables y^t_r in {0, 1} (Section III-A)
        for i in range(len(self.yVarList)):
            for j in range(len(self.yVarList[i])):
                self.yVarList[i][j] = self.newVarID('y', i, j)

    def genNuVarList(self):
        # Topological order literals nu^t_{d,eps} in {0, 1} (Section V-C)
        for i in range(len(self.nuVarList)):
            for j in range(len(self.nuVarList[i])):
                for k in range(len(self.nuVarList[i][j])):
                    self.nuVarList[i][j][k] = self.newVarID('nu', i, j, k)

    def genUVarList(self):
        # Visit sequence variables u^t_v (Section III-A)
        for i in range(len(self.uVarList)):
            for j in range(len(self.uVarList[i])):
                self.uVarList[i][j] = self.newVarID('u', i, j)

    def genHVarList(self):
        # Continuous vehicle load variables h^t_v (Section III-A)
        for i in range(len(self.hVarList)):
            for j in range(len(self.hVarList[i])):
                self.hVarList[i][j] = self.newVarID('h', i, j)

    # ========================== Variable Index Accessors ==========================

    def getLastXVarID(self):
        return self.xVarList[-1][-1][-1]

    def getLastYVarID(self):
        if self.registry is not None:
            y_ids = [vid for key, vid in self.registry.varDict.items() if key[0] == 'y']
            return max(y_ids) if y_ids else 0
        else:
            return self.yVarList[-1][-1]

    def getLastNuVarID(self):
        return self.nuVarList[-1][-1][-1]

    def get_vpool_start_id(self):
        if self.registry is not None:
            return self.registry.get_max_core_id() + 1
        else:
            return self.varID + 1

__all__ = ["PPDSP_reform"]
