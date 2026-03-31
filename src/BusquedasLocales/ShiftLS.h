/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   ShiftLS.h
 * @brief  Implementación de la clase ShiftLS, que representa una búsqueda
 *        local basada en el operador Shift(i, j1, j2) para el problema
 *        MS-CFLP-CI.
 * @author  Keran Miranda González
 * @version 1.0
 * @date 2025-06-01
 */
#pragma once

#include "../Plantillas/Algorithm.h"
#include "../Instancia/MSCFLPInstance.h"
#include "../Solucion/MSCFLPSolution.h"

#include <vector>
#include <limits>
#include <string>

class ShiftLS : public LocalSearch {
public:
    enum class ImprovementStrategy { FIRST_IMPROVEMENT, BEST_IMPROVEMENT };

    explicit ShiftLS(const MSCFLPInstance& inst,
                     ImprovementStrategy strategy = ImprovementStrategy::BEST_IMPROVEMENT)
        : LocalSearch(inst)
        , inst_(inst)
        , strategy_(strategy)
    {}

    std::string getName() const override {
        return strategy_ == ImprovementStrategy::BEST_IMPROVEMENT
               ? "ShiftLS (best)" : "ShiftLS (first)";
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

    struct ShiftMove {
        int i, j1, j2; double q, delta;
    };
    
    bool applyBest(MSCFLPSolution& sol) {
        const int n = inst_.getN();
        const int m = inst_.getM();

        ShiftMove best{-1,-1,-1,0.0,-1e-9};
        bool found = false;

        for (int i = 0; i < n; ++i) {
            const double di = inst_.getDemand(i);

            for (int j1 : sol.getFacilitiesOf(i)) {
                const double assignedInJ1 = sol.getX(i, j1) * di;
                const double cJ1          = inst_.getTransportCost(i, j1);
                const bool   j1Solo       = (sol.getClientsOf(j1).size() == 1);

                for (int j2 = 0; j2 < m; ++j2) {
                    if (j2 == j1) continue;

                    if (!sol.isOpen(j2)) {
                        if (!j1Solo) continue;  
                        if (inst_.getFixedCost(j2) >= inst_.getFixedCost(j1)) continue;
                    }

                    if (sol.getIncompCount(i, j2) != 0) continue;

                    const double rj2 = sol.isOpen(j2)
                                       ? sol.getResidualCap(j2)
                                       : inst_.getCapacity(j2);
                    if (rj2 <= 1e-9) continue;

                    const double q     = std::min(assignedInJ1, rj2);
                    const bool   moveAll = std::abs(assignedInJ1 - q) < 1e-9;

                    double delta = q * (inst_.getTransportCost(i, j2) - cJ1);
                    if (j1Solo && moveAll)   delta -= inst_.getFixedCost(j1);
                    if (!sol.isOpen(j2))     delta += inst_.getFixedCost(j2);

                    if (delta < best.delta) {
                        best = {i, j1, j2, q, delta};
                        found = true;
                    }
                }
            }
        }

        if (!found) return false;
        applyShift(sol, best);
        return true;
    }

    bool applyFirst(MSCFLPSolution& sol) {
        const int n = inst_.getN();
        const int m = inst_.getM();

        for (int i = 0; i < n; ++i) {
            const double di = inst_.getDemand(i);

            for (int j1 : sol.getFacilitiesOf(i)) {
                const double assignedInJ1 = sol.getX(i, j1) * di;
                const double cJ1          = inst_.getTransportCost(i, j1);
                const bool   j1Solo       = (sol.getClientsOf(j1).size() == 1);

                for (int j2 = 0; j2 < m; ++j2) {
                    if (j2 == j1) continue;
                    if (!sol.isOpen(j2)) {
                        if (!j1Solo) continue;
                        if (inst_.getFixedCost(j2) >= inst_.getFixedCost(j1)) continue;
                    }
                    if (sol.getIncompCount(i, j2) != 0) continue;

                    const double rj2 = sol.isOpen(j2)
                                       ? sol.getResidualCap(j2)
                                       : inst_.getCapacity(j2);
                    if (rj2 <= 1e-9) continue;

                    const double q     = std::min(assignedInJ1, rj2);
                    const bool   moveAll = std::abs(assignedInJ1 - q) < 1e-9;

                    double delta = q * (inst_.getTransportCost(i, j2) - cJ1);
                    if (j1Solo && moveAll) delta -= inst_.getFixedCost(j1);
                    if (!sol.isOpen(j2))   delta += inst_.getFixedCost(j2);

                    if (delta < -1e-9) {
                        applyShift(sol, {i, j1, j2, q, delta});
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void applyShift(MSCFLPSolution& sol, const ShiftMove& mv) const {
        if (!sol.isOpen(mv.j2)) sol.openFacility(mv.j2);

        const double di       = inst_.getDemand(mv.i);
        const double totalJ1  = sol.getX(mv.i, mv.j1) * di;
        const bool   moveAll  = std::abs(totalJ1 - mv.q) < 1e-9;

        sol.removeAssignment(mv.i, mv.j1);
        if (sol.getClientsOf(mv.j1).empty()) sol.closeFacility(mv.j1);

        if (!moveAll) {
            const double resto = totalJ1 - mv.q;
            if (!sol.isOpen(mv.j1)) sol.openFacility(mv.j1);
            sol.assignDemand(mv.i, mv.j1, resto);
        }

        sol.assignDemand(mv.i, mv.j2, mv.q);
    }
};