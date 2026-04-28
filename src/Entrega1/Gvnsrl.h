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

class GVNSRL : public Metaheuristic {
public:
    explicit GVNSRL(const MSCFLPInstance& inst,
                    int          graspIter      = 10,
                    int          alpha          = 3,
                    int          beta           = 3,
                    unsigned int seed           = 0,
                    int          maxGVNSIter    = 50,
                    int          shakingK       = 3,
                    double       rlAlpha        = 0.2,
                    double       rlEpsilon      = 0.2,
                    int          maxSinMejora   = 20,
                    int          maxTotalRL     = 100,
                    double       epsilonDecay   = 1.0,   ///< λ decaimiento ε (1.0 = sin decaimiento)
                    bool         propReward     = false, ///< true = recompensa proporcional
                    std::string  qLogFile       = "")    ///< fichero CSV de evolución Q
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
        , rng_(seed == 0
               ? static_cast<unsigned int>(
                     std::chrono::steady_clock::now()
                         .time_since_epoch().count())
               : seed)
    {}

    std::string getName() const override {
        return "GVNS-RL (gvnsIter=" + std::to_string(maxGVNSIter_)
             + ", k="  + std::to_string(shakingK_)
             + ", α="  + std::to_string(rlAlpha_)
             + ", ε0=" + std::to_string(rlEpsilon0_)
             + ", λ="  + std::to_string(epsilonDecay_) + ")";
    }

    /// Devuelve cuántas veces fue seleccionada cada LS (para análisis)
    const std::vector<int>& getSelectionCounts() const { return selectionCounts_; }

protected:
    std::unique_ptr<Solution> solve() override {

        // Solución inicial con GRASP completo
        GRASP grasp(inst_, graspIter_, alpha_, beta_, seed_,
                    ShiftLS::ImprovementStrategy::FIRST_IMPROVEMENT,
                    LocalSearchChoice::ALL);
        auto best    = grasp.run();
        double bestCost = best->getTotalCost();

        // LS con FIRST para el bucle GVNS
        ShiftLS             shiftLS  (inst_, ShiftLS::ImprovementStrategy::FIRST_IMPROVEMENT);
        SwapClientesLS      swapCliLS(inst_, SwapClientesLS::ImprovementStrategy::FIRST_IMPROVEMENT);
        SwapInstalacionesLS swapInstLS(inst_, SwapInstalacionesLS::ImprovementStrategy::FIRST_IMPROVEMENT);
        IncompatElimLS      incompLS (inst_, IncompatElimLS::ImprovementStrategy::FIRST_IMPROVEMENT);

        // LS con BEST para la intensificación final
        ShiftLS             shiftBest  (inst_, ShiftLS::ImprovementStrategy::BEST_IMPROVEMENT);
        SwapClientesLS      swapCliBest(inst_, SwapClientesLS::ImprovementStrategy::BEST_IMPROVEMENT);
        SwapInstalacionesLS swapInstBest(inst_, SwapInstalacionesLS::ImprovementStrategy::BEST_IMPROVEMENT);
        IncompatElimLS      incompBest (inst_, IncompatElimLS::ImprovementStrategy::BEST_IMPROVEMENT);

        // Tabla Q y contadores de selección
        std::vector<double> Q(4, 0.5);
        selectionCounts_.assign(4, 0);

        // Preparar fichero de log CSV si se especificó
        std::ofstream qLog;
        if (!qLogFile_.empty()) {
            qLog.open(qLogFile_);
            if (qLog.is_open())
                qLog << "iteracion,epsilon,Q_Incompat,Q_Shift,Q_SwapInst,Q_SwapCli,"
                     << "sel_Incompat,sel_Shift,sel_SwapInst,sel_SwapCli,coste_actual\n";
        }

        for (iterationsRun_ = 0; iterationsRun_ < maxGVNSIter_; ++iterationsRun_) {

            // Decaimiento de ε: εt = ε0 · λ^t
            double epsilon = rlEpsilon0_
                           * std::pow(epsilonDecay_,
                                      static_cast<double>(iterationsRun_));
            epsilon = std::max(epsilon, 0.01);  // mínimo 1% exploración

            auto sPrime = best->clone();
            shaking(dynamic_cast<MSCFLPSolution&>(*sPrime));
            vndRL(*sPrime, Q, epsilon,
                  shiftLS, swapCliLS, swapInstLS, incompLS);

            if (sPrime->getTotalCost() < bestCost) {
                bestCost = sPrime->getTotalCost();
                best     = sPrime->clone();
                std::fill(Q.begin(), Q.end(), 0.5);
            }

            // Registrar estado Q en el CSV
            if (qLog.is_open()) {
                qLog << std::fixed << std::setprecision(4)
                     << iterationsRun_ << ","
                     << epsilon << ","
                     << Q[0] << "," << Q[1] << "," << Q[2] << "," << Q[3] << ","
                     << selectionCounts_[0] << "," << selectionCounts_[1] << ","
                     << selectionCounts_[2] << "," << selectionCounts_[3] << ","
                     << bestCost << "\n";
            }
        }

        if (qLog.is_open()) qLog.close();

        // Intensificación final con BEST
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
    int          graspIter_;
    int          alpha_;
    int          beta_;
    int          maxGVNSIter_;
    int          shakingK_;
    double       rlAlpha_;
    double       rlEpsilon0_;
    int          maxSinMejora_;
    int          maxTotalRL_;
    double       epsilonDecay_;
    bool         propReward_;
    std::string  qLogFile_;
    std::mt19937 rng_;

    mutable std::vector<int> selectionCounts_;  ///< veces que se eligió cada LS

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
    // VND-RL con ε variable, recompensa proporcional y contadores
    // =========================================================================
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