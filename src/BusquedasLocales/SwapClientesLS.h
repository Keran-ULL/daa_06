/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 *
 * @file   SwapClientesLS.h
 * @brief  Búsqueda local basada en el operador Swap-Clientes(i1, i2).
 * @author Keran Miranda González
 * @version 2.0
 */
#pragma once

#include "../Plantillas/Algorithm.h"
#include "../Instancia/MSCFLPInstance.h"
#include "../Solucion/MSCFLPSolution.h"

#include <vector>
#include <unordered_set>
#include <limits>
#include <string>

class SwapClientesLS : public LocalSearch {
public:
    enum class ImprovementStrategy { FIRST_IMPROVEMENT, BEST_IMPROVEMENT };

    explicit SwapClientesLS(const MSCFLPInstance& inst,
                             ImprovementStrategy strategy = ImprovementStrategy::BEST_IMPROVEMENT)
        : LocalSearch(inst)
        , inst_(inst)
        , strategy_(strategy)
    {}

    std::string getName() const override {
        return strategy_ == ImprovementStrategy::BEST_IMPROVEMENT
               ? "SwapClientesLS (best)" : "SwapClientesLS (first)";
    }

    void setStrategy(ImprovementStrategy s) { strategy_ = s; }

    bool improve(Solution& solution) override {
        return applyBestMove(solution);
    }

    bool applyBestMove(Solution& solution) override {
        auto& sol = dynamic_cast<MSCFLPSolution&>(solution);
        return strategy_ == ImprovementStrategy::BEST_IMPROVEMENT
               ? applyBest(sol) : applyFirst(sol);
    }

private:
    const MSCFLPInstance& inst_;
    ImprovementStrategy   strategy_;

    struct SwapMove {
        int i1, i2, ja, jb; double qa, qb, delta;
    };

    using IncompatSet = std::vector<std::unordered_set<int>>;

    IncompatSet buildIncompatCache() const {
        const int n = inst_.getN();
        IncompatSet cache(n);
        for (int i = 0; i < n; ++i)
            for (int nb : inst_.getIncompatibleWith(i))
                cache[i].insert(nb);
        return cache;
    }

    bool applyBest(MSCFLPSolution& sol) {
        const int m = inst_.getM();

        IncompatSet incompCache = buildIncompatCache();

        SwapMove best{-1,-1,-1,-1,0,0,-1e-9};
        bool found = false;

        for (int ja = 0; ja < m; ++ja) {
            if (!sol.isOpen(ja)) continue;
            const auto& clientsA = sol.getClientsOf(ja);
            if (clientsA.empty()) continue;

            for (int jb = ja + 1; jb < m; ++jb) {
                if (!sol.isOpen(jb)) continue;
                const auto& clientsB = sol.getClientsOf(jb);
                if (clientsB.empty()) continue;

                for (int i1 : clientsA) {
                    const double d1 = inst_.getDemand(i1);
                    const double qa = sol.getX(i1, ja) * d1;

                    for (int i2 : clientsB) {
                        const double d2 = inst_.getDemand(i2);
                        const double qb = sol.getX(i2, jb) * d2;

                        double delta;
                        if (!isFeasible(sol, i1, i2, ja, jb, qa, qb,
                                        incompCache, delta)) continue;
                        if (delta < best.delta) {
                            best = {i1,i2,ja,jb,qa,qb,delta};
                            found = true;
                        }
                    }
                }
            }
        }

        if (!found) return false;
        applySwap(sol, best);
        return true;
    }

    bool applyFirst(MSCFLPSolution& sol) {
        const int m = inst_.getM();
        IncompatSet incompCache = buildIncompatCache();

        for (int ja = 0; ja < m; ++ja) {
            if (!sol.isOpen(ja)) continue;
            const auto& clientsA = sol.getClientsOf(ja);
            if (clientsA.empty()) continue;

            for (int jb = ja + 1; jb < m; ++jb) {
                if (!sol.isOpen(jb)) continue;
                const auto& clientsB = sol.getClientsOf(jb);
                if (clientsB.empty()) continue;

                for (int i1 : clientsA) {
                    const double d1 = inst_.getDemand(i1);
                    const double qa = sol.getX(i1, ja) * d1;

                    for (int i2 : clientsB) {
                        const double d2 = inst_.getDemand(i2);
                        const double qb = sol.getX(i2, jb) * d2;

                        double delta;
                        if (!isFeasible(sol, i1, i2, ja, jb, qa, qb,
                                        incompCache, delta)) continue;
                        if (delta < -1e-9) {
                            applySwap(sol, {i1,i2,ja,jb,qa,qb,delta});
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    bool isFeasible(const MSCFLPSolution& sol,
                    int i1, int i2, int ja, int jb,
                    double qa, double qb,
                    const IncompatSet& incompCache,
                    double& delta) const
    {
        if (sol.getResidualCap(ja) + qa < qb - 1e-9) return false;
        if (sol.getResidualCap(jb) + qb < qa - 1e-9) return false;

        bool i1i2Incompat = (incompCache[i1].count(i2) > 0);
        int adj = i1i2Incompat ? 1 : 0;

        if (sol.getIncompCount(i2, ja) - adj != 0) return false;
        if (sol.getIncompCount(i1, jb) - adj != 0) return false;

        delta = inst_.getDemand(i1) * (inst_.getTransportCost(i1,jb) - inst_.getTransportCost(i1,ja))
              + inst_.getDemand(i2) * (inst_.getTransportCost(i2,ja) - inst_.getTransportCost(i2,jb));
        return true;
    }

    void applySwap(MSCFLPSolution& sol, const SwapMove& mv) const {
        sol.removeAssignment(mv.i1, mv.ja);
        sol.removeAssignment(mv.i2, mv.jb);
        sol.assignDemand(mv.i1, mv.jb, mv.qa);
        sol.assignDemand(mv.i2, mv.ja, mv.qb);
    }
};