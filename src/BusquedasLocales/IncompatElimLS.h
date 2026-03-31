/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 *
 * @file   IncompatElimLS.h
 * @brief  Búsqueda local de eliminación de incompatibilidades.
 * @author Keran Miranda González
 * @version 2.0
 */
#pragma once

#include "../Plantillas/Algorithm.h"
#include "../Instancia/MSCFLPInstance.h"
#include "../Solucion/MSCFLPSolution.h"

#include <vector>
#include <algorithm>
#include <limits>
#include <string>

class IncompatElimLS : public LocalSearch {
public:
    enum class ImprovementStrategy { FIRST_IMPROVEMENT, BEST_IMPROVEMENT };

    explicit IncompatElimLS(const MSCFLPInstance& inst,
                             ImprovementStrategy strategy = ImprovementStrategy::BEST_IMPROVEMENT)
        : LocalSearch(inst)
        , inst_(inst)
        , strategy_(strategy)
    {}

    std::string getName() const override {
        return strategy_ == ImprovementStrategy::BEST_IMPROVEMENT
               ? "IncompatElimLS (best)" : "IncompatElimLS (first)";
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

    struct ElimMove { int i, jSrc, jDest; double q, delta; };

    bool applyBest(MSCFLPSolution& sol) {
        const int m = inst_.getM();

        ElimMove best{-1,-1,-1,0,std::numeric_limits<double>::infinity()};
        bool found = false;

        for (int j = 0; j < m; ++j) {
            if (!sol.isOpen(j)) continue;
            const auto& clients = sol.getClientsOf(j);
            if (clients.empty()) continue;

            int iStar = -1, maxScore = -1;
            for (int i : clients) {
                int score = sol.getIncompCount(i, j);  // O(1)
                if (score > maxScore) { maxScore = score; iStar = i; }
            }
            if (iStar == -1 || maxScore == 0) continue;

            ElimMove mv;
            if (!findBestDest(sol, iStar, j, mv)) continue;
            if (mv.delta < best.delta) { best = mv; found = true; }
        }

        if (!found || best.delta >= -1e-9) return false;
        executeMove(sol, best);
        return true;
    }

    bool applyFirst(MSCFLPSolution& sol) {
        const int m = inst_.getM();

        for (int j = 0; j < m; ++j) {
            if (!sol.isOpen(j)) continue;
            const auto& clients = sol.getClientsOf(j);
            if (clients.empty()) continue;

            int iStar = -1, maxScore = -1;
            for (int i : clients) {
                int score = sol.getIncompCount(i, j);  // O(1)
                if (score > maxScore) { maxScore = score; iStar = i; }
            }
            if (iStar == -1 || maxScore == 0) continue;

            ElimMove mv;
            if (!findBestDest(sol, iStar, j, mv)) continue;
            if (mv.delta < -1e-9) { executeMove(sol, mv); return true; }
        }
        return false;
    }
    
    bool findBestDest(const MSCFLPSolution& sol,
                      int iStar, int jSrc, ElimMove& mv) const
    {
        const int    m     = inst_.getM();
        const double di    = inst_.getDemand(iStar);
        const double qSrc  = sol.getX(iStar, jSrc) * di;
        const double cSrc  = inst_.getTransportCost(iStar, jSrc);
        const bool   jSrcSolo = (sol.getClientsOf(jSrc).size() == 1);

        mv.delta = std::numeric_limits<double>::infinity();
        bool found = false;

        for (int jd = 0; jd < m; ++jd) {
            if (jd == jSrc) continue;
            if (sol.getIncompCount(iStar, jd) != 0) continue;

            const bool isOpen = sol.isOpen(jd);
            const double rjd  = isOpen ? sol.getResidualCap(jd)
                                       : inst_.getCapacity(jd);
            if (rjd < qSrc - 1e-9) continue;

            double delta = qSrc * (inst_.getTransportCost(iStar, jd) - cSrc);
            if (jSrcSolo)  delta -= inst_.getFixedCost(jSrc);
            if (!isOpen)   delta += inst_.getFixedCost(jd);

            if (delta < mv.delta) {
                mv = {iStar, jSrc, jd, qSrc, delta};
                found = true;
            }
        }
        return found && mv.delta < -1e-9;
    }

    void executeMove(MSCFLPSolution& sol, const ElimMove& mv) const {
        if (!sol.isOpen(mv.jDest)) sol.openFacility(mv.jDest);
        sol.removeAssignment(mv.i, mv.jSrc);
        if (sol.getClientsOf(mv.jSrc).empty()) sol.closeFacility(mv.jSrc);
        sol.assignDemand(mv.i, mv.jDest, mv.q);
    }
};