// PPDSP_utils.cc
// Embedded CEGAR-based structural Lazy Clause Generation (LCG) callbacks for UWrMaxSat
// (Algorithm 1, Section IV, Section V-D, Section VI-B3).

#include "PPDSP_utils.h"
#include <algorithm>
#include <iostream>
#include <cassert>
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include "gurobi_c++.h"

extern const char* opt_ppdsp_cut_type;

using namespace std;

double PPDSP_start_time = 0;
int    PPDSP_time_limit = -1;

// Construct inverse variable lookup table mapping propositional variable IDs to x^t_{od} and y^t_r (Section III-A)
void PPDSP_utils::buildVarIndexMap(PPDSP_Instance* inst)
{
    inst->id2Var.clear();
    // Binary routing variables x^t_{od}
    for (int t=0; t < inst->lenOfVehicle; t++){
        for (size_t o=0; o < inst->xVarList[t].size(); o++){
            for (size_t d=0; d < inst->xVarList[t][o].size(); d++){
                int vid = inst->xVarList[t][o][d];
                inst->id2Var[vid] = VarInfo('x', t, o, d);
            }
        }
    }
    // Binary request assignment variables y^t_r
    for (int r=0; r < inst->lenOfRequest; r++){
        for (int t=0; t < inst->lenOfVehicle; t++){
            int vid = inst->yVarList[r][t];
            inst->id2Var[vid] = VarInfo('y', r, t, -1);
        }
    }
}

// Filter propositional truth assignment to retain positive routing and selection literals (x^t_{od}, y^t_r)
void PPDSP_utils::extractXYModel(
    PPDSP_Instance* inst,
    const std::vector<int>& fullModel,
    std::vector<int>& xyModel
){
    xyModel.clear();
    int maxY = inst->getLastYVarID();
    for (int lit : fullModel){
        if (lit > 0 && lit <= maxY)
            xyModel.push_back(lit);
    }
}

// Reconstruct candidate vehicle routes R^t and selected request sets S from a truth assignment (Section IV-A)
void PPDSP_utils::decodeModel(
    PPDSP_Instance* inst,
    const std::vector<int>& xyModel,
    std::vector<std::vector<std::pair<int,int>>>& vehRoutes,
    std::vector<std::vector<int>>& vehReqs
){
    vehRoutes.assign(inst->lenOfVehicle, {});
    vehReqs.assign(inst->lenOfVehicle, {});

    if (inst->id2Var.empty())
        buildVarIndexMap(inst);

    // Extract active arcs (o, d) and assigned requests r for each vehicle t in T
    for (int vid : xyModel){
        std::unordered_map<int,VarInfo>::const_iterator it = inst->id2Var.find(vid);
        if (it == inst->id2Var.end()) continue;

        const VarInfo &info = it->second;

        if (info.type == 'x'){
            int t = info.a;
            int o = info.b;
            int d = info.c;
            if (o != d)
                vehRoutes[t].push_back(std::make_pair(o,d));
        }
        else if (info.type == 'y'){
            int r = info.a;
            int t = info.b;
            vehReqs[t].push_back(r);
        }
    }

    // Reconstruct ordered arc sequence R^t originating from the depot (index lenOfLocation)
    int depot = inst->lenOfLocation;
    for (int t=0; t<inst->lenOfVehicle; t++){
        std::vector<std::pair<int,int> > &edges = vehRoutes[t];
        if (edges.empty()) continue;

        std::unordered_map<int,int> nxt;
        for (size_t i = 0; i < edges.size(); i++)
            nxt[edges[i].first] = edges[i].second;

        std::vector<std::pair<int,int> > route;
        int cur = depot;
        while (nxt.count(cur)){
            int d = nxt[cur];
            route.push_back(std::make_pair(cur, d));
            cur = d;
            if (cur == depot) break;
        }
        vehRoutes[t] = route;
    }
}

// Sequential capacity evaluation, Minimal Conflict Set (MCS) extraction, and structural LCG clause generation
// (Algorithm 1, Proposition 1, Section V-D1 Eq. (20), and Section V-D2 Eq. (21))
bool PPDSP_utils::checkOverload(
    PPDSP_Instance* inst,
    int vehID,
    const std::vector<std::pair<int,int>>& route,
    const std::vector<int>& assigned_reqs,
    std::vector<std::vector<Minisat::Lit>>& learnt_clauses,
    bool enable_broadcast
){
    if (route.empty()) return false;

    int capacity = inst->vehicleList[vehID][0];
    int load = 0;

    vector<int> req_size(inst->lenOfRequest);
    vector<int> pickup(inst->lenOfRequest);
    vector<int> drop(inst->lenOfRequest);

    for (int r = 0; r < inst->lenOfRequest; r++){
        req_size[r] = inst->requestList[r][1];
        pickup[r]   = inst->requestList[r][2];
        drop[r]     = inst->requestList[r][3];
    }

    vector<bool> onboard(inst->lenOfRequest, false);

    // Simulate candidate route R^t and track accumulated load Gamma and onboard set S_onboard (Algorithm 1, Lines 2-12)
    for (size_t k = 0; k < route.size(); k++){
        int d = route[k].second;

        // Update accumulated load Gamma and onboard request set S_onboard at destination node d
        for (int r : assigned_reqs){
            if (d == pickup[r] && d == drop[r]) {
                continue;
            }
            else if (d == pickup[r]){
                load += req_size[r];
                onboard[r] = true;
            }
            else if (d == drop[r] && onboard[r]){
                load -= req_size[r];
                onboard[r] = false;
            }
        }

        // Capacity violation Gamma > c detected at arc index k (Algorithm 1, Line 13)
        if (load > capacity){
            learnt_clauses.clear();

            // 1. Collect active onboard requests S_onboard
            vector<int> onboard_reqs;
            for (int r : assigned_reqs)
                if (onboard[r])
                    onboard_reqs.push_back(r);

            // 2. Sort S_onboard in descending order of demand q_r (Algorithm 1, Line 14; Proposition 1)
            std::sort(onboard_reqs.begin(), onboard_reqs.end(),
                [&](int a, int b){
                    return req_size[a] > req_size[b];
                }
            );

            // 3. Greedily accumulate requests into the Minimal Conflict Set (MCS) S_conflict until Gamma_sub > c (Algorithm 1, Lines 15-20)
            vector<int> minimal_conflict;
            int current_subset_load = 0;
            for (int r : onboard_reqs) {
                current_subset_load += req_size[r];
                minimal_conflict.push_back(r);
                if (current_subset_load > capacity) {
                    break;
                }
            }

            // 4. Broadcast generated conflict clauses across all homogeneous vehicles t' in T (Section IV-D; Algorithm 1, Lines 21-23)
            const auto& group = inst->identicalVehicles[vehID];
            const std::vector<int>* target_vehicles = nullptr;
            std::vector<int> single_vehicle_group;

            if (enable_broadcast) {
                target_vehicles = &group;
            } else {
                single_vehicle_group.push_back(vehID);
                target_vehicles = &single_vehicle_group;
            }

            // ========================================================
            // Structural Lazy Clause Generation (Section V-D)
            // ========================================================
            if (opt_ppdsp_cut_type != nullptr && std::strcmp(opt_ppdsp_cut_type, "path") == 0) {
                // ----------------------------------------------------
                // Path-Based LCG (MaxSAT-path): Section V-D1, Eq. (20)
                // bigvee_{r in S_conflict} ~y^t_r vee bigvee_{(i,j) in E_prefix} ~x^t_{ij}
                // ----------------------------------------------------
                for (int targetID : *target_vehicles) {
                    std::vector<Minisat::Lit> clause;
                    
                    // Negated assignment literals ~y^t_r for r in S_conflict
                    for (int r : minimal_conflict) {
                        int vid = inst->yVarList[r][targetID];
                        if (vid != 0) clause.push_back(~Minisat::mkLit(vid - 1));
                    }
                    
                    // Negated routing arc literals ~x^t_{ij} for (i, j) in E_prefix
                    for (size_t step = 0; step <= k; step++) {
                        int src = route[step].first;
                        int tgt = route[step].second;
                        int x_vid = inst->xVarList[targetID][src][tgt];
                        if (x_vid != 0) {
                            clause.push_back(~Minisat::mkLit(x_vid - 1));
                        }
                    }

                    std::sort(clause.begin(), clause.end());
                    clause.erase(std::unique(clause.begin(), clause.end()), clause.end());
                    learnt_clauses.push_back(std::move(clause));
                }
            } else {
                // ----------------------------------------------------
                // Order-Based LCG (MaxSAT-order): Section V-D2, Eq. (21), Theorems 2 & 3
                // bigvee_{r in S_conflict} (~y^t_r vee ~nu^t_{f(r),eps} vee nu^t_{g(r),eps})
                // for all t in T and eps in {0, ..., |V|-3}
                // ----------------------------------------------------
                for (int targetID : *target_vehicles) {
                    // Positional index eps ranges from 0 to |V|-3 (where lenOfLocation = |V|-1, so max_p = |V|-2)
                    int max_p = inst->lenOfLocation - 2; 

                    for (int p = 0; p < max_p; p++) {
                        std::vector<Minisat::Lit> clause;
                        
                        // a) Negated request assignment literals ~y^t_r for r in S_conflict
                        for (int r : minimal_conflict) {
                            int vid = inst->yVarList[r][targetID];
                            if (vid != 0) clause.push_back(~Minisat::mkLit(vid - 1));
                        }
                        
                        // b) Negated pickup order literals ~nu^t_{f(r),eps} for r in S_conflict
                        for (int r : minimal_conflict) {
                            int pickupNode = pickup[r];
                            int nu_vid = inst->nuVarList[targetID][pickupNode][p];
                            if (nu_vid != 0) {
                                clause.push_back(~Minisat::mkLit(nu_vid - 1));
                            }
                        }
                        
                        // c) Positive delivery order literals nu^t_{g(r),eps} for r in S_conflict
                        for (int r : minimal_conflict) {
                            int dropNode = drop[r];
                            int nu_vid = inst->nuVarList[targetID][dropNode][p];
                            if (nu_vid != 0) {
                                clause.push_back(Minisat::mkLit(nu_vid - 1));
                            }
                        }
                        
                        std::sort(clause.begin(), clause.end());
                        clause.erase(std::unique(clause.begin(), clause.end()), clause.end());
                        
                        learnt_clauses.push_back(std::move(clause));
                    }
                }
            }
            return true;
        }
    }
    return false;
}

// Auxiliary LP subproblem evaluator for time-window feasibility via Farkas dual ray extraction
bool PPDSP_utils::checkTimeWindow(
    PPDSP_Instance* inst,
    int vehID,
    const std::vector<std::pair<int,int>>& route,
    const std::vector<int>& assigned_reqs,
    std::vector<std::vector<Minisat::Lit>>& learnt_clauses,
    bool enable_broadcast
){
    if (route.empty()) return false;

    try {
        // 1. Initialize silent global Gurobi environment
        static GRBEnv* env = nullptr;
        if (env == nullptr) {
            std::cout << "c [Debug] Initializing global Gurobi Environment for the first time..." << std::endl;
            env = new GRBEnv(true);
            env->set("LogFile", "");
            env->set(GRB_IntParam_OutputFlag, 0);
            env->start();
        }
        GRBModel model = GRBModel(*env);

        // Enable Farkas dual ray extraction and disable dual presolve reductions
        model.set(GRB_IntParam_InfUnbdInfo, 1);
        model.set(GRB_IntParam_DualReductions, 0);

        // 2. Convert directed arc sequence into an ordered node visitation sequence
        std::vector<int> nodes;
        nodes.push_back(route[0].first);
        for (const auto& edge : route) {
            nodes.push_back(edge.second);
        }

        // 3. Create continuous arrival time variables T_i for each visited node
        std::vector<GRBVar> T(nodes.size());
        for (size_t i = 0; i < nodes.size(); i++) {
            T[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "T_" + std::to_string(i));
        }

        // 4. Map linear constraints to routing arcs (u, v) and assigned requests r for conflict cut generation
        std::vector<std::pair<GRBConstr, std::pair<int,int>>> x_constrs; // Mapped to arc (u, v)
        std::vector<std::pair<GRBConstr, int>> y_constrs;                // Mapped to request r

        // 5. Traverse visited nodes to impose time-window bounds and temporal propagation constraints
        for (size_t i = 0; i < nodes.size(); i++) {
            int node = nodes[i];
            int sum_service = 0;

            // Identify all pickup and delivery operations occurring at the current physical location
            for (int r : assigned_reqs) {
                int pk = inst->requestList[r][2];
                int dp = inst->requestList[r][3];
                int p_s = inst->requestList[r][4];
                int p_e = inst->requestList[r][5];
                int p_l = inst->requestList[r][6];
                int d_s = inst->requestList[r][7];
                int d_e = inst->requestList[r][8];
                int d_l = inst->requestList[r][9];

                // Pickup operation time-window bounds
                if (node == pk) {
                    sum_service += p_s;
                    y_constrs.push_back({model.addConstr(T[i] >= p_e), r});
                    y_constrs.push_back({model.addConstr(T[i] <= p_l), r});
                }
                // Delivery operation time-window bounds
                if (node == dp) {
                    sum_service += d_s;
                    y_constrs.push_back({model.addConstr(T[i] >= d_e), r});
                    y_constrs.push_back({model.addConstr(T[i] <= d_l), r});
                }
            }

            // Temporal propagation constraint along the traversed arc: T_i + sum_service + travel_time <= T_{i+1}
            if (i < nodes.size() - 1) {
                int nxt_node = nodes[i+1];
                int travel_time = inst->locaList[node][nxt_node];
                x_constrs.push_back({model.addConstr(T[i] + sum_service + travel_time <= T[i+1]), {node, nxt_node}});
            }
        }

        std::cout << "c [Debug] Veh " << vehID << " building LP done. Nodes: " << nodes.size() << ". Optimizing... " << std::flush;
        // 6. Solve the linear programming feasibility subproblem
        model.optimize();
        std::cout << "Done! Status: " << model.get(GRB_IntAttr_Status) << std::endl;

        // 7. Extract Farkas proof of infeasibility to construct a combinatorial Benders conflict clause
        if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
            learnt_clauses.clear();

            std::vector<int> single_vehicle_group = {vehID};
            const std::vector<int>& target_vehicles = enable_broadcast ? inst->identicalVehicles[vehID] : single_vehicle_group;

            for (int targetID : target_vehicles) {
                std::vector<Minisat::Lit> clause;
                
                for (const auto& pair : x_constrs) {
                    if (std::abs(pair.first.get(GRB_DoubleAttr_FarkasDual)) > 1e-5) {
                        int x_var = inst->xVarList[targetID][pair.second.first][pair.second.second];
                        if (x_var != 0) clause.push_back(~Minisat::mkLit(x_var - 1));
                    }
                }
                
                for (const auto& pair : y_constrs) {
                    if (std::abs(pair.first.get(GRB_DoubleAttr_FarkasDual)) > 1e-5) {
                        int y_var = inst->yVarList[pair.second][targetID];
                        if (y_var != 0) clause.push_back(~Minisat::mkLit(y_var - 1));
                    }
                }

                std::sort(clause.begin(), clause.end());
                clause.erase(std::unique(clause.begin(), clause.end()), clause.end());

                learnt_clauses.push_back(std::move(clause));
            }
            return true;
        }
    } catch(GRBException& e) {
        std::cerr << "[Gurobi LP Error] " << e.getMessage() << std::endl;
    }
    return false; // Feasible schedule
}

// Parse instance metadata (.meta) exported by PPDSP_utils.export_meta (Section V-D, Section VI-B3)
bool loadPPDSPInstance(const char* filename, PPDSP_Instance& inst)
{
    std::ifstream in(filename);
    if (!in.good()) {
        std::cerr << "[UWrMaxSAT] Cannot open meta file: " << filename << std::endl;
        return false;
    }

    // Read header dimensions: |T| (lenOfVehicle), |R| (lenOfRequest), |V|-1 (lenOfLocation)
    in >> inst.lenOfVehicle >> inst.lenOfRequest >> inst.lenOfLocation;

    // Allocate variable index tables and instance parameter arrays
    inst.xVarList.assign(inst.lenOfVehicle,
                         std::vector<std::vector<int>>(inst.lenOfLocation+1,
                                                       std::vector<int>(inst.lenOfLocation+1, 0)));
    inst.yVarList.assign(inst.lenOfRequest,
                         std::vector<int>(inst.lenOfVehicle, 0));
    inst.nuVarList.assign(inst.lenOfVehicle,
                          std::vector<std::vector<int>>(inst.lenOfLocation+1,
                                                        std::vector<int>(inst.lenOfLocation+1, 0)));
    inst.requestList.assign(inst.lenOfRequest,
                            std::vector<int>(10, 0));
    inst.vehicleList.assign(inst.lenOfVehicle,
                            std::vector<int>(2, 0));
    inst.locaList.assign(inst.lenOfLocation + 1,
                         std::vector<int>(inst.lenOfLocation + 1, 0));
    inst.identicalVehicles.assign(inst.lenOfVehicle, {});
    for(int i=0; i<inst.lenOfVehicle; ++i) inst.identicalVehicles[i].push_back(i);

    std::string tag;
    std::string section = "";

    while (in >> tag)
    {
        // Switch active metadata section header
        if (tag == "#") {
            in >> section; // Section identifier (e.g., xVarList, yVarList, nuVarList)

            // Parse travel cost matrix l_{od} (locaList) if present
            if (section == "locaList") {
                for (int i = 0; i <= inst.lenOfLocation; ++i) {
                    for (int j = 0; j <= inst.lenOfLocation; ++j) {
                        in >> inst.locaList[i][j];
                    }
                }
            }
            continue;
        }

        // Parse entries for the active metadata section
        if (section == "xVarList") {
            int t = std::stoi(tag);
            int o, d, vid;
            in >> o >> d >> vid;
            inst.xVarList[t][o][d] = vid;
        }
        else if (section == "yVarList") {
            int r = std::stoi(tag);
            int t, vid;
            in >> t >> vid;
            inst.yVarList[r][t] = vid;
        }
        else if (section == "nuVarList") {
            int t = std::stoi(tag);
            int d, eps, vid;
            in >> d >> eps >> vid;
            inst.nuVarList[t][d][eps] = vid;
        }
        else if (section == "requestList") {
            int r = std::stoi(tag);
            std::string line;
            std::getline(in, line); 
            std::stringstream ss(line);
            int w = 0, q = 0, pk = 0, dp = 0;
            int p_s = 0, p_e = 0, p_l = 0, d_s = 0, d_e = 0, d_l = 0;
            if (ss >> w >> q >> pk >> dp) {
                if (ss >> p_s) {
                    ss >> p_e >> p_l >> d_s >> d_e >> d_l;
                }
            }
            inst.requestList[r] = {w, q, pk, dp, p_s, p_e, p_l, d_s, d_e, d_l};
        }
        else if (section == "vehicleList") {
            int t = std::stoi(tag);
            double cap, cost;
            in >> cap >> cost;
            inst.vehicleList[t] = {(int)cap, (int)cost};
        }
        else if (section == "vehicleGroups") {
            // int gid = std::stoi(tag);
            int count;
            in >> count;
            std::vector<int> groupMembers;
            for(int k=0; k<count; ++k) {
                int vid;
                in >> vid;
                groupMembers.push_back(vid);
            }
            // Register homogeneous vehicle symmetry group for cut broadcasting (Section IV-D)
            for(int vid : groupMembers) {
                inst.identicalVehicles[vid] = groupMembers;
            }
        }
    }

    return true;
}

// Load assumption literals from file into a Minisat literal vector
bool loadAssumptionLits(const char* filename, Minisat::vec<Minisat::Lit>& out_assumps, int maxVarID)
{
    std::ifstream in(filename);
    if (!in.good()){
        std::cerr << "[UWrMaxSAT] Cannot open assumption file: " << filename << std::endl;
        return false;
    }
    
    std::string tok;
    while (in >> tok){
        // Parse signed integer literal (e.g., 12 or -45)
        int lit = std::stoi(tok);
        int v = std::abs(lit);

        if (v < 1 || v > maxVarID){
            std::cerr << "[UWrMaxSAT] WARNING: assumption literal "
                      << lit << " out of range 1.." << maxVarID << std::endl;
            continue;
        }

        Minisat::Lit L = Minisat::mkLit(v-1, lit < 0);
        out_assumps.push(L);
    }

    std::cout << "[UWrMaxSAT] Loaded " << out_assumps.size()
              << " assumption literals from " << filename << std::endl;

    return true;
}

void PPDSP_setTimeLimit(int seconds) { PPDSP_time_limit = seconds; }

void PPDSP_startTimer() { PPDSP_start_time = std::clock(); }

bool PPDSP_timeout() {
    if (PPDSP_time_limit <= 0) return false;
    double elapsed = (std::clock() - PPDSP_start_time) / CLOCKS_PER_SEC;
    return elapsed >= PPDSP_time_limit;
}
