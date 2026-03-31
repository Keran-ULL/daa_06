/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   Greedy.h
 * @brief  Implementación de la clase Greedy, que representa un algoritmo
 *       constructivo voraz para el problema MS-CFLP-CI.
 * @author  Keran Miranda González
 * @version 1.0
 */

 #pragma once

#include "../Plantillas/Algorithm.h"
#include "../Instancia/MSCFLPInstance.h"
#include "../Solucion/MSCFLPSolution.h"

#include <vector>
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <string>

class Greedy : public ConstructiveAlgorithm {
public:
    static constexpr int DEFAULT_SLACK = 5;  ///< Parámetro k 

    /**
     * @param  instance  Instancia del MS-CFLP-CI ya cargada.
     * @param  slack     Número de instalaciones extra de holgura (k).
     *                   Por defecto 5 (valor del guión).
     * @throws std::invalid_argument si la instancia no es de tipo MSCFLPInstance.
     */
    explicit Greedy(const MSCFLPInstance& instance, int slack = DEFAULT_SLACK)
        : ConstructiveAlgorithm(instance)
        , inst_(instance)
        , slack_(slack)
    {
        if (slack_ < 0)
            throw std::invalid_argument("Greedy: el parámetro de holgura k no puede ser negativo");
    }

    std::string getName() const override {
        return "Greedy (k=" + std::to_string(slack_) + ")";
    }

    /** @brief Modifica el parámetro de holgura k en tiempo de ejecución. */
    void setSlack(int k) {
        if (k < 0) throw std::invalid_argument("Greedy::setSlack: k no puede ser negativo");
        slack_ = k;
    }

    int getSlack() const { return slack_; }

protected:

    std::unique_ptr<Solution> solve() override {
      auto sol = std::make_unique<MSCFLPSolution>(inst_);
      phase1SelectFacilities(*sol);
      phase2AssignClients(*sol);
      // Evaluación final: recalcula costes descompuestos y feasible_
      sol->evaluate();
      return sol;
    }

private:
    const MSCFLPInstance& inst_;
    int slack_;

    void phase1SelectFacilities(MSCFLPSolution& sol) {
        const int m = inst_.getM();
        const int n = inst_.getN();
        // --- 1: Ordenar F por f_j ascendente ---
        std::vector<int> sortedFacilities(m);
        std::iota(sortedFacilities.begin(), sortedFacilities.end(), 0);
        std::sort(sortedFacilities.begin(), sortedFacilities.end(),
                  [&](int a, int b) {
                      return inst_.getFixedCost(a) < inst_.getFixedCost(b);
                  });

        // --- 2: Dtotal = Σ d_i ---
        double Dtotal = 0.0;
        for (int i = 0; i < n; ++i)
            Dtotal += inst_.getDemand(i);

        // --- 3: Abrir instalaciones hasta cubrir Dtotal ---
        double capacidadAcumulada = 0.0;
        int    nextIdx = 0;  // puntero por la lista ordenada
        while (nextIdx < m && capacidadAcumulada < Dtotal) {
            int j = sortedFacilities[nextIdx++];
            sol.openFacility(j);
            capacidadAcumulada += inst_.getCapacity(j);
        }

        // --- 4: Añadir k instalaciones extra de holgura ---
        int extraAdded = 0;
        while (nextIdx < m && extraAdded < slack_) {
            int j = sortedFacilities[nextIdx++];
            sol.openFacility(j);
            ++extraAdded;
        }
        // Nota: si quedan menos de k instalaciones sin abrir, se añaden todas.
    }

    void phase2AssignClients(MSCFLPSolution& sol) {
        const int n = inst_.getN();
        const int m = inst_.getM();
        // Construir la lista de instalaciones abiertas una sola vez
        std::vector<int> openFacilities;
        openFacilities.reserve(m);
        for (int j = 0; j < m; ++j)
            if (sol.isOpen(j))
                openFacilities.push_back(j);
        // --- Para cada cliente i ---
        for (int i = 0; i < n; ++i) {
            const double di = inst_.getDemand(i);
            double       demandaRestante = di;
            // --- Fopen ordenado por c_{ij} ascendente ---
            std::vector<int> Li = openFacilities;
            std::sort(Li.begin(), Li.end(),
                      [&](int a, int b) {
                          return inst_.getTransportCost(i, a)
                               < inst_.getTransportCost(i, b);
                      });
            for (int j : Li) {
                if (demandaRestante <= 0.0) break;
                // --- Verificar compatibilidad ---
                if (sol.getIncompCount(i, j) != 0) continue;
                const double rj = sol.getResidualCap(j);
                if (rj <= 0.0) continue;
                // --- Asignar q = min(d_i_restante, r_j) ---
                const double q = std::min(demandaRestante, rj);
                sol.assignDemand(i, j, q);   // actualiza x, w, residualCap,
                                             // clientsOf, facilitiesOf,
                                             // incompCount y transportCost
                demandaRestante -= q;
            }
            // Si demandaRestante > 0 aquí, el cliente i no ha sido cubierto
            // completamente → la solución final será infactible.
        }
    }
};