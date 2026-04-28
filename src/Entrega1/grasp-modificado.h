/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 *
 * @file   GRASP_modificado.h
 * @brief  GRASP con las dos modificaciones solicitadas en la tarea.
 * @author Keran Miranda González
 * @version 5.0 — Modificación
 *
 * ═══════════════════════════════════════════════════════════════════════
 * MODIFICACIONES RESPECTO A GRASP.h (v4.0)
 * ═══════════════════════════════════════════════════════════════════════
 *
 * MODIFICACIÓN 1 — Solución inicial (líneas marcadas con [MOD-1])
 * ───────────────────────────────────────────────────────────────
 * ANTES: la solución de partida de cada iteración multiarranque era la
 *        devuelta por GRASPConstructive (fase constructiva pura, sin mejora).
 *
 * AHORA: la solución de partida es la devuelta por el algoritmo GRASP con
 *        mejora única de SwapClientesLS. Es decir, antes de entrar en el
 *        RVND reducido se ejecuta un bucle SwapClientes hasta convergencia,
 *        obteniendo así un punto de partida de mayor calidad.
 *
 * MODIFICACIÓN 2 — RVND reducido (líneas marcadas con [MOD-2])
 * ─────────────────────────────────────────────────────────────
 * ANTES: el RVND usaba cuatro estructuras de vecindad:
 *        {IncompatElim, Shift, SwapInstalaciones, SwapClientes}
 *
 * AHORA: el RVND usa únicamente dos estructuras:
 *        {Shift, SwapInstalaciones}
 *        Se eliminan IncompatElimLS y SwapClientesLS del bucle RVND.
 *        La lógica de reinicio y eliminación es idéntica a la original,
 *        adaptada al conjunto reducido {0=Shift, 1=SwapInst}.
 *
 * El resto del fichero (constructores, VND, vndRL, lsName, etc.)
 * permanece sin cambios para facilitar la comparación.
 * ═══════════════════════════════════════════════════════════════════════
 */
#pragma once

#include "../Plantillas/Algorithm.h"
#include "../Instancia/MSCFLPInstance.h"
#include "../Solucion/MSCFLPSolution.h"
#include "../Entrega1/GRASPConstructive.h"
#include "../Entrega1/ShiftLS.h"
#include "../Entrega1/SwapClientesLS.h"
#include "../Entrega1/SwapInstalacionesLS.h"
#include "../Entrega1/IncompatElimLS.h"

#include <memory>
#include <string>
#include <limits>
#include <vector>
#include <random>
#include <numeric>
#include <chrono>

enum class LocalSearchChoice {
    SHIFT     = 1,
    SWAP_CLI  = 2,
    SWAP_INST = 3,
    INCOMPAT  = 4,
    ALL       = 5,   ///< RVND (modificado: solo Shift + SwapInst)
    VND       = 6,   ///< VND orden fijo
    GVNS_RL   = 7    ///< GVNS con Reinforcement Learning
};

class GRASP : public Metaheuristic {
public:
    explicit GRASP(const MSCFLPInstance& inst,
                   int               maxIterations  = 30,
                   int               alpha          = 3,
                   int               beta           = 3,
                   unsigned int      seed           = 0,
                   ShiftLS::ImprovementStrategy strategy
                       = ShiftLS::ImprovementStrategy::FIRST_IMPROVEMENT,
                   LocalSearchChoice lsChoice       = LocalSearchChoice::ALL,
                   int               swapInstFreq   = 3,
                   double            rlAlpha        = 0.2,
                   double            rlEpsilon      = 0.2,
                   int               maxSinMejora   = 20,
                   int               maxTotalIter   = 100)
        : Metaheuristic(inst, maxIterations, seed)
        , inst_(inst)
        , alpha_(alpha)
        , beta_(beta)
        , lsStrategy_(strategy)
        , lsChoice_(lsChoice)
        , swapInstFreq_(swapInstFreq)
        , rlAlpha_(rlAlpha)
        , rlEpsilon_(rlEpsilon)
        , maxSinMejora_(maxSinMejora)
        , maxTotalIter_(maxTotalIter)
        , rvndRng_(seed == 0
                   ? static_cast<unsigned int>(
                         std::chrono::steady_clock::now()
                             .time_since_epoch().count())
                   : seed)
    {}

    std::string getName() const override {
        return "GRASP-MOD (iter=" + std::to_string(maxIterations_)
             + ", alpha=" + std::to_string(alpha_)
             + ", beta="  + std::to_string(beta_)
             + ", ls="    + lsName()
             + ") [MOD: inicio=SwapCli, RVND={Shift,SwapInst}]";
    }

    int getAlpha() const { return alpha_; }
    int getBeta()  const { return beta_;  }

protected:
    std::unique_ptr<Solution> solve() override {

        // LS construidas una sola vez fuera del bucle de iteraciones.
        ShiftLS             shiftLS  (inst_, lsStrategy_);
        SwapClientesLS      swapCliLS(inst_,
            static_cast<SwapClientesLS::ImprovementStrategy>(
                static_cast<int>(lsStrategy_)));
        SwapInstalacionesLS swapInstLS(inst_,
            static_cast<SwapInstalacionesLS::ImprovementStrategy>(
                static_cast<int>(lsStrategy_)));
        IncompatElimLS      incompLS (inst_,
            static_cast<IncompatElimLS::ImprovementStrategy>(
                static_cast<int>(lsStrategy_)));

        std::unique_ptr<Solution> best    = nullptr;
        double                    bestCost = std::numeric_limits<double>::infinity();

        for (iterationsRun_ = 0; iterationsRun_ < maxIterations_; ++iterationsRun_) {

            unsigned int iterSeed = seed_ + static_cast<unsigned int>(iterationsRun_);

            // ─────────────────────────────────────────────────────────────
            // [MOD-1] SOLUCIÓN INICIAL
            // ANTES: GRASPConstructive puro (solo fase constructiva).
            // AHORA: constructivo + converge con SwapClientesLS hasta
            //        que no hay mejora → solución de mayor calidad inicial.
            // ─────────────────────────────────────────────────────────────
            GRASPConstructive constructive(inst_, alpha_, beta_, iterSeed);
            auto sol = constructive.run();

            // Mejora inicial exclusiva con SwapClientes hasta convergencia
            {
                bool improved = true;
                while (improved) {
                    improved = swapCliLS.improve(*sol);   // [MOD-1]
                }
            }
            // ─────────────────────────────────────────────────────────────

            const bool applySwapInst = (swapInstFreq_ <= 0) ||
                                       (iterationsRun_ % swapInstFreq_ == 0);

            const bool isBest      = (lsStrategy_ == ShiftLS::ImprovementStrategy::BEST_IMPROVEMENT);
            const int  maxFirstIter = 10;
            int        localIter    = 0;
            bool       anyImproved  = true;

            while (anyImproved && (isBest || localIter < maxFirstIter)) {
                ++localIter;

                switch (lsChoice_) {
                    case LocalSearchChoice::SHIFT:
                        anyImproved = shiftLS.improve(*sol);
                        break;
                    case LocalSearchChoice::SWAP_CLI:
                        anyImproved = swapCliLS.improve(*sol);
                        break;
                    case LocalSearchChoice::SWAP_INST:
                        anyImproved = swapInstLS.improve(*sol);
                        break;
                    case LocalSearchChoice::INCOMPAT:
                        anyImproved = incompLS.improve(*sol);
                        break;
                    case LocalSearchChoice::ALL:
                        // [MOD-2] RVND reducido: solo {Shift, SwapInst}
                        anyImproved = rvndReducido(*sol, applySwapInst,
                                                   shiftLS, swapInstLS);
                        break;
                    case LocalSearchChoice::VND:
                        anyImproved = vnd(*sol, applySwapInst,
                                          shiftLS, swapCliLS, swapInstLS, incompLS);
                        break;
                    case LocalSearchChoice::GVNS_RL:
                        anyImproved = vndRL(*sol, applySwapInst,
                                            shiftLS, swapCliLS, swapInstLS, incompLS);
                        break;
                }

                if (sol->getTotalCost() < bestCost) {
                    bestCost = sol->getTotalCost();
                    best     = sol->clone();
                }
            }
        }

        return best;
    }

private:
    // =========================================================================
    // [MOD-2] RVND REDUCIDO: solo Shift y SwapInstalaciones
    //
    // ANTES (rvnd original): disponibles = {0=Incompat, 1=Shift,
    //                                        2=SwapInst, 3=SwapCli}
    //
    // AHORA: disponibles = {0=Shift, 1=SwapInst}
    //
    // La lógica es idéntica a la original:
    //   - Al mejorar: reiniciar la lista completa {0, 1}.
    //   - Al fallar:  eliminar el vecindario de la lista.
    //   - Termina cuando la lista queda vacía.
    //
    // Se eliminan IncompatElimLS y SwapClientesLS porque:
    //   - IncompatElim: las incompatibilidades ya se han resuelto en la
    //     solución inicial (SwapCli las elimina implícitamente).
    //   - SwapClientes: se usa como mejora inicial exclusiva [MOD-1],
    //     no dentro del RVND.
    // =========================================================================
    bool rvndReducido(Solution&            sol,
                      bool                 applySwapInst,
                      ShiftLS&             shiftLS,
                      SwapInstalacionesLS& swapInstLS)
    {
        // Solo dos vecindarios: 0=Shift, 1=SwapInstalaciones
        std::vector<int> available  = {0, 1};   // [MOD-2]
        bool             anyImproved = false;

        while (!available.empty()) {
            std::uniform_int_distribution<int> dist(
                0, static_cast<int>(available.size()) - 1);
            int idx    = dist(rvndRng_);
            int chosen = available[idx];

            bool improved = false;
            switch (chosen) {
                case 0: improved = shiftLS.improve(sol);                    break; // [MOD-2]
                case 1: improved = applySwapInst
                                   ? swapInstLS.improve(sol) : false;       break; // [MOD-2]
            }

            if (improved) {
                anyImproved = true;
                // Reiniciar la lista completa {Shift, SwapInst}
                available = {0, 1};                                          // [MOD-2]
            } else {
                available.erase(available.begin() + idx);
            }
        }
        return anyImproved;
    }

    // =========================================================================
    // RVND original (sin modificar) — se mantiene para referencia
    // Se usa solo si lsChoice_ != ALL (los otros modos no se modifican)
    // =========================================================================
    bool rvnd(Solution&            sol,
              bool                 applySwapInst,
              ShiftLS&             shiftLS,
              SwapClientesLS&      swapCliLS,
              SwapInstalacionesLS& swapInstLS,
              IncompatElimLS&      incompLS)
    {
        std::vector<int> available    = {0, 1, 2};
        bool             anyImproved  = false;
        bool             cheapImproved = false;

        while (!available.empty()) {
            std::uniform_int_distribution<int> dist(
                0, static_cast<int>(available.size()) - 1);
            int idx    = dist(rvndRng_);
            int chosen = available[idx];

            bool improved = false;
            switch (chosen) {
                case 0: improved = incompLS.improve(sol);                  break;
                case 1: improved = shiftLS.improve(sol);                   break;
                case 2: improved = applySwapInst
                                   ? swapInstLS.improve(sol) : false;     break;
                case 3: improved = swapCliLS.improve(sol);                 break;
            }

            if (improved) {
                anyImproved = true;
                if (chosen != 3) cheapImproved = true;
                available = cheapImproved
                            ? std::vector<int>{0, 1, 2, 3}
                            : std::vector<int>{0, 1, 2};
            } else {
                available.erase(available.begin() + idx);
                if (available.empty() && cheapImproved) {
                    available.push_back(3);
                    cheapImproved = false;
                }
            }
        }
        return anyImproved;
    }

    // =========================================================================
    // VND orden fijo (sin modificar)
    // =========================================================================
    bool vnd(Solution&            sol,
             bool                 applySwapInst,
             ShiftLS&             shiftLS,
             SwapClientesLS&      swapCliLS,
             SwapInstalacionesLS& swapInstLS,
             IncompatElimLS&      incompLS)
    {
        const int nNeighborhoods = 4;
        bool anyImproved   = false;
        bool cheapImproved = false;
        int  k = 0;

        while (k < nNeighborhoods) {
            if (k == 3 && !cheapImproved) break;

            bool improved = false;
            switch (k) {
                case 0: improved = incompLS.improve(sol);                  break;
                case 1: improved = shiftLS.improve(sol);                   break;
                case 2: improved = applySwapInst
                                   ? swapInstLS.improve(sol) : false;     break;
                case 3: improved = swapCliLS.improve(sol);                 break;
            }

            if (improved) {
                anyImproved = true;
                if (k != 3) cheapImproved = true;
                k = 0;
            } else {
                ++k;
            }
        }
        return anyImproved;
    }

    // =========================================================================
    // VND-RL (sin modificar)
    // =========================================================================
    bool vndRL(Solution&            sol,
               bool                 applySwapInst,
               ShiftLS&             shiftLS,
               SwapClientesLS&      swapCliLS,
               SwapInstalacionesLS& swapInstLS,
               IncompatElimLS&      incompLS)
    {
        std::vector<double> Q(4, 0.5);
        std::uniform_real_distribution<double> realDist(0.0, 1.0);
        std::uniform_int_distribution<int>     idxDist(0, 3);

        bool anyImproved = false;
        int  sinMejora   = 0;
        int  totalIter   = 0;

        while (sinMejora < maxSinMejora_ && totalIter < maxTotalIter_) {
            ++totalIter;

            int chosen;
            if (realDist(rvndRng_) < rlEpsilon_)
                chosen = idxDist(rvndRng_);
            else
                chosen = static_cast<int>(
                    std::max_element(Q.begin(), Q.end()) - Q.begin());

            bool improved = false;
            switch (chosen) {
                case 0: improved = incompLS.improve(sol);                  break;
                case 1: improved = shiftLS.improve(sol);                   break;
                case 2: improved = applySwapInst
                                   ? swapInstLS.improve(sol) : false;     break;
                case 3: improved = swapCliLS.improve(sol);                 break;
            }

            double r = improved ? 1.0 : 0.0;
            Q[chosen] += rlAlpha_ * (r - Q[chosen]);

            if (improved) { anyImproved = true; sinMejora = 0; }
            else          { ++sinMejora; }
        }
        return anyImproved;
    }

    const MSCFLPInstance&        inst_;
    int                          alpha_;
    int                          beta_;
    ShiftLS::ImprovementStrategy lsStrategy_;
    LocalSearchChoice            lsChoice_;
    int                          swapInstFreq_;
    double                       rlAlpha_;
    double                       rlEpsilon_;
    int                          maxSinMejora_;
    int                          maxTotalIter_;
    std::mt19937                 rvndRng_;

    std::string lsName() const {
        switch (lsChoice_) {
            case LocalSearchChoice::SHIFT:     return "Shift";
            case LocalSearchChoice::SWAP_CLI:  return "SwapClientes";
            case LocalSearchChoice::SWAP_INST: return "SwapInstalaciones";
            case LocalSearchChoice::INCOMPAT:  return "IncompatElim";
            case LocalSearchChoice::ALL:       return "RVND-Reducido{Shift,SwapInst}";
            case LocalSearchChoice::VND:       return "VND";
            case LocalSearchChoice::GVNS_RL:   return "GVNS-RL";
        }
        return "?";
    }
};