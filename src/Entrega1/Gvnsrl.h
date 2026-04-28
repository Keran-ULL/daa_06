/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 *
 * @file   GVNSRL.h
 * @brief  GVNS con Reinforcement Learning para el MS-CFLP-CI.
 * @author Keran Miranda González
 * @version 1.0
 *
 * ESTRUCTURA
 * ──────────
 * S ← GRASP()                      ← solución inicial de calidad
 * while no criterio de parada:
 *     S' ← Shaking(S, k)           ← perturbación: cierra k instalaciones
 *     S' ← VND-RL(S')              ← mejora con Q-Learning
 *     if f(S') < f(S): S ← S'      ← aceptar si mejora
 * return S
 *
 * SHAKING
 * ───────
 * Cierra k instalaciones elegidas al azar y reasigna sus clientes
 * a otras instalaciones abiertas usando criterio voraz (menor c[i][j]).
 * Esto perturba suficientemente la solución para escapar de óptimos locales
 * sin destruirla completamente.
 *
 * VND-RL
 * ──────
 * Selección ε-greedy sobre tabla Q de tamaño 4 (una entrada por LS).
 * Actualización: Q(LSk) ← Q(LSk) + α·[r - Q(LSk)]
 * La tabla Q se reinicia en cada iteración del bucle GVNS para que
 * el aprendizaje sea local a cada perturbación.
 */

#ifndef GVNSRL_H
#define GVNSRL_H
#include "../Plantillas/Algorithm.h"
#include "../Instancia/MSCFLPInstance.h"
#include "../Solucion/MSCFLPSolution.h"
#include "../Algoritmo/GraspConstructive.h"
#include "../BusquedasLocales/ShiftLS.h"
#include "../BusquedasLocales/SwapClientesLS.h"
#include "../BusquedasLocales/SwapInstalacionesLS.h"
#include "../BusquedasLocales/IncompatElimLS.h"
#include "../Entrega1/GRASP.h"

#include <memory>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <limits>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <array>

/**
 * @brief Estrategia de mejora interna del bucle GVNS.
 *
 * Cambiar el modo es tan sencillo como pasar un valor distinto al constructor
 * o al menú. Todas las variantes comparten el mismo Shaking y la misma
 * intensificación final BEST.
 *
 *  RL_VND      – Q-Learning ε-greedy (comportamiento original)
 *  SEQUENTIAL  – Incompat → Shift → SwapInst → SwapCli (una pasada completa)
 *  VND_FIXED   – orden fijo con reinicio al mejorar (VND clásico)
 *  RVND        – orden aleatorio con reinicio al mejorar
 */
enum class GVNSImprovMode {
    RL_VND     = 1,   ///< Q-Learning ε-greedy          (original)
    SEQUENTIAL = 2,   ///< Secuencial sin reinicio
    VND_FIXED  = 3,   ///< VND orden fijo con reinicio
    RVND       = 4    ///< VND orden aleatorio con reinicio
};

class GVNSRL : public Metaheuristic {
public:
    explicit GVNSRL(const MSCFLPInstance& inst,
                    int            graspIter    = 10,
                    int            alpha        = 3,
                    int            beta         = 3,
                    unsigned int   seed         = 0,
                    int            maxGVNSIter  = 50,
                    int            shakingK     = 3,
                    double         rlAlpha      = 0.2,
                    double         rlEpsilon    = 0.2,
                    int            maxSinMejora = 20,
                    int            maxTotalRL   = 100,
                    double         epsilonDecay = 1.0,
                    bool           propReward   = false,
                    std::string    qLogFile     = "",
                    GVNSImprovMode mode         = GVNSImprovMode::RL_VND)
        : Metaheuristic(inst, maxGVNSIter, seed)
        , inst_(inst)
        , graspIter_(graspIter)
        , alpha_(alpha)
        , beta_(beta)
        , maxGVNSIter_(maxGVNSIter)
        , shakingK_(shakingK)
        , rlAlpha_(rlAlpha)
        , rlEpsilon0_(rlEpsilon)
        , maxSinMejora_(maxSinMejora)
        , maxTotalRL_(maxTotalRL)
        , epsilonDecay_(epsilonDecay)
        , propReward_(propReward)
        , qLogFile_(qLogFile)
        , mode_(mode)
        , rng_(seed == 0
               ? static_cast<unsigned int>(
                     std::chrono::steady_clock::now()
                         .time_since_epoch().count())
               : seed)
    {}

    std::string getName() const override {
        std::string modeName;
        switch (mode_) {
            case GVNSImprovMode::RL_VND:     modeName = "RL-VND";      break;
            case GVNSImprovMode::SEQUENTIAL: modeName = "Sequential";   break;
            case GVNSImprovMode::VND_FIXED:  modeName = "VND-Fixed";    break;
            case GVNSImprovMode::RVND:       modeName = "RVND";         break;
        }
        return "GVNS[" + modeName + "]"
             + " iter=" + std::to_string(maxGVNSIter_)
             + " k="    + std::to_string(shakingK_)
             + " a="    + std::to_string(rlAlpha_)
             + " e0="   + std::to_string(rlEpsilon0_);
    }

    /// Devuelve cuántas veces fue seleccionada cada LS (para análisis)
    const std::vector<int>& getSelectionCounts() const { return selectionCounts_; }

protected:
    std::unique_ptr<Solution> solve() override {

        GRASP grasp(inst_, graspIter_, alpha_, beta_, seed_,
                    ShiftLS::ImprovementStrategy::FIRST_IMPROVEMENT,
                    LocalSearchChoice::ALL);
        auto best      = grasp.run();
        double bestCost = best->getTotalCost();

        // LS con FIRST para el bucle GVNS (rápidas → más iteraciones)
        ShiftLS             shiftLS  (inst_, ShiftLS::ImprovementStrategy::FIRST_IMPROVEMENT);
        SwapClientesLS      swapCliLS(inst_, SwapClientesLS::ImprovementStrategy::FIRST_IMPROVEMENT);
        SwapInstalacionesLS swapInstLS(inst_, SwapInstalacionesLS::ImprovementStrategy::FIRST_IMPROVEMENT);
        IncompatElimLS      incompLS (inst_, IncompatElimLS::ImprovementStrategy::FIRST_IMPROVEMENT);

        // LS con BEST solo para la intensificación final
        ShiftLS             shiftBest  (inst_, ShiftLS::ImprovementStrategy::BEST_IMPROVEMENT);
        SwapClientesLS      swapCliBest(inst_, SwapClientesLS::ImprovementStrategy::BEST_IMPROVEMENT);
        SwapInstalacionesLS swapInstBest(inst_, SwapInstalacionesLS::ImprovementStrategy::BEST_IMPROVEMENT);
        IncompatElimLS      incompBest (inst_, IncompatElimLS::ImprovementStrategy::BEST_IMPROVEMENT);

        std::vector<double> Q(4, 0.5);
        selectionCounts_.assign(4, 0);

        std::ofstream qLog;
        if (!qLogFile_.empty()) {
            qLog.open(qLogFile_);
            if (qLog.is_open())
                qLog << "iteracion,epsilon,Q_Incompat,Q_Shift,Q_SwapInst,Q_SwapCli,"
                     << "sel_Incompat,sel_Shift,sel_SwapInst,sel_SwapCli,coste_actual\n";
        }

        for (iterationsRun_ = 0; iterationsRun_ < maxGVNSIter_; ++iterationsRun_) {

            double epsilon = rlEpsilon0_
                           * std::pow(epsilonDecay_,
                                      static_cast<double>(iterationsRun_));
            epsilon = std::max(epsilon, 0.01);

            auto sPrime = best->clone();
            shaking(dynamic_cast<MSCFLPSolution&>(*sPrime));

            switch (mode_) {
                case GVNSImprovMode::RL_VND:
                    vndRL(*sPrime, Q, epsilon,
                          shiftLS, swapCliLS, swapInstLS, incompLS);
                    break;
                case GVNSImprovMode::SEQUENTIAL:
                    sequential(*sPrime,
                               shiftLS, swapCliLS, swapInstLS, incompLS);
                    break;
                case GVNSImprovMode::VND_FIXED:
                    vndFixed(*sPrime,
                             shiftLS, swapCliLS, swapInstLS, incompLS);
                    break;
                case GVNSImprovMode::RVND:
                    rvnd(*sPrime,
                         shiftLS, swapCliLS, swapInstLS, incompLS);
                    break;
            }

            if (sPrime->getTotalCost() < bestCost) {
                bestCost = sPrime->getTotalCost();
                best     = sPrime->clone();
                std::fill(Q.begin(), Q.end(), 0.5);
            }

            if (qLog.is_open()) {
                qLog << std::fixed << std::setprecision(4)
                     << iterationsRun_ << "," << epsilon << ","
                     << Q[0] << "," << Q[1] << "," << Q[2] << "," << Q[3] << ","
                     << selectionCounts_[0] << "," << selectionCounts_[1] << ","
                     << selectionCounts_[2] << "," << selectionCounts_[3] << ","
                     << bestCost << "\n";
            }
        }

        if (qLog.is_open()) qLog.close();

        // Intensificación final BEST
        bool anyImproved = true;
        while (anyImproved) {
            bool i1 = incompBest.improve(*best);
            bool i2 = shiftBest.improve(*best);
            bool i3 = swapInstBest.improve(*best);
            bool i4 = (i1||i2||i3) ? swapCliBest.improve(*best) : false;
            anyImproved = i1 || i2 || i3 || i4;
        }

        return best;
    }

private:
    const MSCFLPInstance& inst_;
    int            graspIter_;
    int            alpha_;
    int            beta_;
    int            maxGVNSIter_;
    int            shakingK_;
    double         rlAlpha_;
    double         rlEpsilon0_;
    int            maxSinMejora_;
    int            maxTotalRL_;
    double         epsilonDecay_;
    bool           propReward_;
    std::string    qLogFile_;
    GVNSImprovMode mode_;
    std::mt19937   rng_;

    mutable std::vector<int> selectionCounts_;

    // =========================================================================
    // Shaking
    // =========================================================================
    void shaking(MSCFLPSolution& sol) {
        const int m = inst_.getM();
        std::vector<int> opened;
        for (int j = 0; j < m; ++j)
            if (sol.isOpen(j)) opened.push_back(j);
        if (static_cast<int>(opened.size()) <= 1) return;

        int k = std::min(shakingK_, static_cast<int>(opened.size()) - 1);
        std::shuffle(opened.begin(), opened.end(), rng_);

        std::vector<bool> closing(m, false);
        for (int t = 0; t < k; ++t) closing[opened[t]] = true;

        for (int t = 0; t < k; ++t) {
            int jClose = opened[t];
            if (!sol.isOpen(jClose)) continue;

            std::vector<int> clients = sol.getClientsOf(jClose);
            for (int i : clients) {
                if (!sol.isServedBy(i, jClose)) continue;
                double demRest = sol.getX(i, jClose) * inst_.getDemand(i);
                sol.removeAssignment(i, jClose);

                std::vector<std::pair<double,int>> cands;
                for (int j2 = 0; j2 < m; ++j2) {
                    if (!sol.isOpen(j2) || closing[j2]) continue;
                    if (sol.getIncompCount(i,j2) != 0)  continue;
                    double rj = sol.getResidualCap(j2);
                    if (rj > 1e-9)
                        cands.emplace_back(inst_.getTransportCost(i,j2), j2);
                }
                std::sort(cands.begin(), cands.end());

                for (auto& [cost, j2] : cands) {
                    if (demRest <= 1e-9) break;
                    double q = std::min(demRest, sol.getResidualCap(j2));
                    sol.assignDemand(i, j2, q);
                    demRest -= q;
                }

                if (demRest > 1e-9) {
                    int bestJ = -1;
                    double bestF = std::numeric_limits<double>::infinity();
                    for (int j2 = 0; j2 < m; ++j2) {
                        if (sol.isOpen(j2) || sol.getIncompCount(i,j2) != 0) continue;
                        if (inst_.getCapacity(j2) < demRest - 1e-9) continue;
                        if (inst_.getFixedCost(j2) < bestF) {
                            bestF = inst_.getFixedCost(j2); bestJ = j2;
                        }
                    }
                    if (bestJ != -1) {
                        sol.openFacility(bestJ);
                        sol.assignDemand(i, bestJ, demRest);
                    }
                }
            }
            if (sol.getClientsOf(jClose).empty())
                sol.closeFacility(jClose);
        }
    }

    // =========================================================================
    // MODO 2: Secuencial — una pasada completa en orden fijo sin reinicio
    //   Incompat → Shift → SwapInst → SwapCli
    //   Útil como baseline para comparar con el VND-RL.
    // =========================================================================
    void sequential(Solution&            sol,
                    ShiftLS&             shiftLS,
                    SwapClientesLS&      swapCliLS,
                    SwapInstalacionesLS& swapInstLS,
                    IncompatElimLS&      incompLS)
    {
        int sinMejora = 0;
        int totalIter = 0;

        // Orden fijo: 0=Incompat, 1=Shift, 2=SwapInst, 3=SwapCli
        const std::array<int,4> ORDER = {0, 1, 2, 3};

        while (sinMejora < maxSinMejora_ && totalIter < maxTotalRL_) {
            ++totalIter;
            bool anyImproved = false;

            for (int chosen : ORDER) {
                bool improved = false;
                switch (chosen) {
                    case 0: improved = incompLS.applyBestMove(sol);  break;
                    case 1: improved = shiftLS.applyBestMove(sol);   break;
                    case 2: improved = swapInstLS.applyBestMove(sol);break;
                    case 3: improved = swapCliLS.applyBestMove(sol); break;
                }
                ++selectionCounts_[chosen];
                if (improved) anyImproved = true;
            }

            if (anyImproved) sinMejora = 0;
            else             ++sinMejora;
        }
    }

    // =========================================================================
    // MODO 3: VND orden fijo con reinicio al mejorar
    //   Al mejorar con LSk se vuelve a LS0; al fallar se avanza a LSk+1.
    // =========================================================================
    void vndFixed(Solution&            sol,
                  ShiftLS&             shiftLS,
                  SwapClientesLS&      swapCliLS,
                  SwapInstalacionesLS& swapInstLS,
                  IncompatElimLS&      incompLS)
    {
        // Orden fijo: Incompat(0) → Shift(1) → SwapInst(2) → SwapCli(3)
        int k         = 0;
        int totalIter = 0;

        while (k < 4 && totalIter < maxTotalRL_) {
            ++totalIter;
            bool improved = false;
            switch (k) {
                case 0: improved = incompLS.applyBestMove(sol);  break;
                case 1: improved = shiftLS.applyBestMove(sol);   break;
                case 2: improved = swapInstLS.applyBestMove(sol);break;
                case 3: improved = swapCliLS.applyBestMove(sol); break;
            }
            ++selectionCounts_[k];

            if (improved) k = 0;   // reiniciar al principio
            else          ++k;     // avanzar al siguiente vecindario
        }
    }

    // =========================================================================
    // MODO 4: RVND — orden aleatorio con reinicio al mejorar
    //   Similar al VND fijo pero el orden se baraja en cada reinicio.
    // =========================================================================
    // MODIFICACIÓN
        void rvnd(Solution&            sol,
              ShiftLS&             shiftLS,
              SwapClientesLS&      swapCliLS,
              SwapInstalacionesLS& swapInstLS,
              IncompatElimLS&      incompLS)
    {
        std::vector<int> available = {0, 1};   
        std::shuffle(available.begin(), available.end(), rng_);
        int totalIter = 0;
 
        while (!available.empty() && totalIter < maxTotalRL_) {
            ++totalIter;
            int k = available[0];
 
            bool improved = false;
            switch (k) {
                case 0: improved = shiftLS.applyBestMove(sol);    break;
                case 1: improved = swapInstLS.applyBestMove(sol); break;
            }
            ++selectionCounts_[k];
 
            if (improved) {
                // Reiniciar con los dos vecindarios en orden aleatorio
                available = {0, 1};
                std::shuffle(available.begin(), available.end(), rng_);
            } else {
                // Eliminar el vecindario que no mejoró
                available.erase(available.begin());
            }
        }
    }


    // =========================================================================
    // MODO 1 (original): VND-RL con Q-Learning ε-greedy
    void vndRL(Solution&            sol,
               std::vector<double>& Q,
               double               epsilon,
               ShiftLS&             shiftLS,
               SwapClientesLS&      swapCliLS,
               SwapInstalacionesLS& swapInstLS,
               IncompatElimLS&      incompLS)
    {
        std::uniform_real_distribution<double> realDist(0.0, 1.0);
        std::uniform_int_distribution<int>     idxDist(0, 3);

        int sinMejora = 0;
        int totalIter = 0;

        while (sinMejora < maxSinMejora_ && totalIter < maxTotalRL_) {
            ++totalIter;

            // Selección ε-greedy con ε variable (decaimiento aplicado fuera)
            int chosen;
            if (realDist(rng_) < epsilon) {
                chosen = idxDist(rng_);
            } else {
                chosen = static_cast<int>(
                    std::max_element(Q.begin(), Q.end()) - Q.begin());
            }
            ++selectionCounts_[chosen];

            double costeBefore = sol.getTotalCost();
            bool   improved    = false;
            switch (chosen) {
                case 0: improved = incompLS.applyBestMove(sol);  break;
                case 1: improved = shiftLS.applyBestMove(sol);   break;
                case 2: improved = swapInstLS.applyBestMove(sol);break;
                case 3: improved = swapCliLS.applyBestMove(sol); break;
            }

            // Recompensa: proporcional o binaria
            double r;
            if (improved) {
                if (propReward_ && costeBefore > 1e-9)
                    r = (costeBefore - sol.getTotalCost()) / costeBefore;
                else
                    r = 1.0;
            } else {
                r = 0.0;
            }

            // Q(LSk) ← Q(LSk) + α·[r − Q(LSk)]
            Q[chosen] += rlAlpha_ * (r - Q[chosen]);

            if (improved) sinMejora = 0;
            else          ++sinMejora;
        }
    }
};

#endif // GVNSRL_H