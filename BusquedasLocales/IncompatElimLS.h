/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   IncompatElimLS.h
 * @brief  Implementación de la clase IncompatElimLS, que representa una
 *        búsqueda local de eliminación de incompatibilidades para el problema
 *       MS-CFLP-CI.
 * @author  Keran Miranda González
 * @version 1.0
 * @date 2025-06-01
 */
#pragma once

#include "Algorithm.h"
#include "MSCFLPInstance.h"
#include "MSCFLPSolution.h"

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
               ? "IncompatElimLS (best)"
               : "IncompatElimLS (first)";
    }

    void setStrategy(ImprovementStrategy s) { strategy_ = s; }

    bool improve(Solution& solution) override {
        auto& sol = dynamic_cast<MSCFLPSolution&>(solution);
        bool anyImprovement = false;
        bool improved = true;
        while (improved) {
            improved = applyBestMove(sol);
            if (improved) anyImprovement = true;
        }
        return anyImprovement;
    }

    bool applyBestMove(Solution& solution) override {
        auto& sol = dynamic_cast<MSCFLPSolution&>(solution);
        return strategy_ == ImprovementStrategy::BEST_IMPROVEMENT
               ? applyBest(sol)
               : applyFirst(sol);
    }

private:
    const MSCFLPInstance& inst_;
    ImprovementStrategy   strategy_;

    struct ElimMove {
        int    i;       // cliente bloqueante a mover
        int    jSrc;    // instalación origen
        int    jDest;   // instalación destino
        double q;       // cantidad a reasignar
        double delta;   // ganancia (negativo = mejora)
    };

    bool applyBest(MSCFLPSolution& sol) {
        const int m = inst_.getM();
        ElimMove best;
        best.delta = -1e-9;
        bool found = false;
        for (int j = 0; j < m; ++j) {
            if (!sol.isOpen(j)) continue;
            const auto& clients = sol.getClientsOf(j);
            if (clients.empty()) continue;
            // Seleccionar cliente bloqueante i* en j
            int    iStar = -1;
            int    maxScore = -1;
            for (int i : clients) {
                int score = blockingScore(sol, i, j);
                if (score > maxScore) {
                    maxScore = score;
                    iStar = i;
                }
            }
            // Si nadie bloquea (score == 0), este operador no ayuda en j
            if (iStar == -1 || maxScore == 0) continue;
            // Buscar mejor destino para i*
            ElimMove mv;
            if (!findBestDest(sol, iStar, j, mv)) continue;
            if (mv.delta < best.delta) {
                best = mv;
                found = true;
            }
        }
        if (!found) return false;
        executeMove(sol, best);
        return true;
    }

    bool applyFirst(MSCFLPSolution& sol) {
        const int m = inst_.getM();
        for (int j = 0; j < m; ++j) {
            if (!sol.isOpen(j)) continue;
            const auto& clients = sol.getClientsOf(j);
            if (clients.empty()) continue;
            // Cliente de mayor score de bloqueo
            int iStar = -1, maxScore = -1;
            for (int i : clients) {
                int score = blockingScore(sol, i, j);
                if (score > maxScore) { maxScore = score; iStar = i; }
            }
            if (iStar == -1 || maxScore == 0) continue;
            ElimMove mv;
            if (!findBestDest(sol, iStar, j, mv)) continue;
            if (mv.delta < -1e-9) {
                executeMove(sol, mv);
                return true;
            }
        }
        return false;
    }

    int blockingScore(const MSCFLPSolution& sol, int i, int j) const {
        // incompCount[i'][j] para cada i' incompatible con i presente en j.
        // Equivalente al número de incompatibles de i que están en j:
        int score = 0;
        for (int nb : inst_.getIncompatibleWith(i))
            if (sol.isServedBy(nb, j)) ++score;
        return score;
    }

    bool findBestDest(const MSCFLPSolution& sol,
                      int iStar, int jSrc,
                      ElimMove& mv) const
    {
        const int    m       = inst_.getM();
        const double di      = inst_.getDemand(iStar);
        const double qSrc    = sol.getX(iStar, jSrc) * di;
        const double cSrc    = inst_.getTransportCost(iStar, jSrc);

        mv.delta = std::numeric_limits<double>::infinity();
        bool found = false;

        for (int jd = 0; jd < m; ++jd) {
            if (jd == jSrc) continue;
            // Compatibilidad con el destino
            if (sol.getIncompCount(iStar, jd) != 0) continue;
            // Capacidad suficiente para toda la demanda de i* en jSrc
            double rjd = sol.isOpen(jd)
                         ? sol.getResidualCap(jd)
                         : inst_.getCapacity(jd);
            if (rjd < qSrc - 1e-9) continue;

            double delta = qSrc * (inst_.getTransportCost(iStar, jd) - cSrc);

            // Coste fijo: ¿jSrc queda vacía? ¿jd estaba cerrada?
            bool jSrcEmpty = (sol.getClientsOf(jSrc).size() == 1);  // solo iStar
            if (jSrcEmpty)         delta -= inst_.getFixedCost(jSrc);
            if (!sol.isOpen(jd))   delta += inst_.getFixedCost(jd);

            if (delta < mv.delta) {
                mv = {iStar, jSrc, jd, qSrc, delta};
                found = true;
            }
        }
        return found;
    }

    void executeMove(MSCFLPSolution& sol, const ElimMove& mv) const {
        // Abrir destino si estaba cerrado
        if (!sol.isOpen(mv.jDest))
            sol.openFacility(mv.jDest);
        // Retirar toda la asignación de i en jSrc
        sol.removeAssignment(mv.i, mv.jSrc);
        // Cerrar jSrc si quedó vacía
        if (sol.getClientsOf(mv.jSrc).empty())
            sol.closeFacility(mv.jSrc);
        // Asignar en jDest
        sol.assignDemand(mv.i, mv.jDest, mv.q);
    }
};
