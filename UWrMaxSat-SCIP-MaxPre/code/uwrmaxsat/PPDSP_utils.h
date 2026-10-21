#ifndef PPDSP_UTILS_H
#define PPDSP_UTILS_H

#include <vector>
#include <unordered_map>
#include <utility>
#include "PbSolver.h"
#include <ctime>

struct VarInfo {
    char type;   // 'x' or 'y'
    int  a;      // for 'x': vehicle t, for 'y': request r
    int  b;      // for 'x': origin  o, for 'y': vehicle t
    int  c;      // for 'x': dest    d, for 'y': unused (-1)

    VarInfo() : type(0), a(0), b(0), c(0) {}
    VarInfo(char ty, int a_, int b_, int c_) : type(ty), a(a_), b(b_), c(c_) {}
};

// forward declare your PPDSP problem class
class PPDSP_Instance {
public:
    int lenOfVehicle;
    int lenOfRequest;
    int lenOfLocation;

    std::vector<std::vector<std::vector<int>>> xVarList;     // [t][o][d]   = varID
    std::vector<std::vector<int>> yVarList;                  // [r][t]      = varID
    std::vector<std::vector<std::vector<int>>> nuVarList;    // [t][d][eps] = varID

    std::vector<std::vector<int>> requestList;               // [r] = [profit, size, pickup, drop]
    std::vector<std::vector<int>> vehicleList;               // [t] = [capacity, cost_coeff]

    std::vector<std::vector<int>> locaList;                  // [o][d] = travel time

    std::unordered_map<int, VarInfo> id2Var;                 // varID -> (x/y, indices)

    std::vector<std::vector<int>> identicalVehicles;         // [i]: grouped by identical vehicle capacity

    int getLastYVarID() const {
        int r = lenOfRequest - 1;
        int t = lenOfVehicle - 1;
        return yVarList[r][t];
    }
};

class PPDSP_utils {
public:
    static void buildVarIndexMap(PPDSP_Instance* inst);

    static void extractXYModel(
        PPDSP_Instance* inst,
        const std::vector<int>& fullModel,
        std::vector<int>& xyModel
    );

    static void decodeModel(
        PPDSP_Instance* inst,
        const std::vector<int>& xyModel,
        std::vector<std::vector<std::pair<int,int>>>& vehRoutes,
        std::vector<std::vector<int>>& vehReqs
    );

    static bool checkOverload(
        PPDSP_Instance* inst,
        int vehID,
        const std::vector<std::pair<int,int>>& route,
        const std::vector<int>& assigned_reqs,
        std::vector<std::vector<Minisat::Lit>>& learnt_clauses,
        bool enable_broadcast = false
    );

    static bool checkTimeWindow(
        PPDSP_Instance* inst,
        int vehID,
        const std::vector<std::pair<int,int>>& route,
        const std::vector<int>& assigned_reqs,
        std::vector<std::vector<Minisat::Lit>>& learnt_clauses,
        bool enable_broadcast = false
    );
};

// Load PPDSP instance meta from text file
bool loadPPDSPInstance(const char* filename, PPDSP_Instance& inst);

// Load PPDSP assumption literals from text file
bool loadAssumptionLits(const char* filename, Minisat::vec<Minisat::Lit>& out_assumps, int maxVarID);

// Global variables for time management
extern double PPDSP_start_time;
extern int    PPDSP_time_limit;

void PPDSP_setTimeLimit(int seconds);
void PPDSP_startTimer();
bool PPDSP_timeout();

#endif
