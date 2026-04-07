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
#include <random>
#include <stdexcept>
#include <string>
#include <limits>
#include <cmath>

class GRASPConstructive : public ConstructiveAlgorithm {
public:
    static constexpr int DEFAULT_ALPHA = 3;   ///< Tamaño LRC fase 1 (instalaciones)
    static constexpr int DEFAULT_BETA  = 3;   ///< Tamaño LRC fase 2 (asignaciones)

    /**
     * @param  instance  Instancia del MS-CFLP-CI ya cargada.
     * @param  alpha     Tamaño de la LRC en la selección de instalaciones.
     * @param  beta      Tamaño de la LRC en la asignación de clientes.
     * @param  seed      Semilla del RNG. 0 = semilla basada en tiempo.
     */
    explicit GRASPConstructive(const MSCFLPInstance& instance,
                               int          alpha = DEFAULT_ALPHA,
                               int          beta  = DEFAULT_BETA,
                               unsigned int seed  = 0)
        : ConstructiveAlgorithm(instance)
        , inst_(instance)
        , alpha_(alpha)
        , beta_(beta)
        , rng_(seed == 0
               ? static_cast<unsigned int>(
                     std::chrono::steady_clock::now()
                         .time_since_epoch().count())
               : seed)
    {
        validateParams();
    }

    std::string getName() const override {
        return "GRASP-Constructive (alpha=" + std::to_string(alpha_) +
               ", beta=" + std::to_string(beta_) + ")";
    }

    void setAlpha(int alpha) { alpha_ = alpha; validateParams(); }
    void setBeta (int beta)  { beta_  = beta;  validateParams(); }
    void setSeed (unsigned int seed) {
        rng_.seed(seed == 0
                  ? static_cast<unsigned int>(
                        std::chrono::steady_clock::now()
                            .time_since_epoch().count())
                  : seed);
    }

    int getAlpha() const { return alpha_; }
    int getBeta()  const { return beta_;  }

    /**
     * @brief  Interfaz ConstructiveAlgorithm: alpha actúa como tamaño global
     *         de LRC (se asigna a alpha_ y beta_ simultáneamente).
     */
    void setRandomizationParam(int param) override {
        alpha_ = param;
        beta_  = param;
        validateParams();
    }

protected:

    std::unique_ptr<Solution> solve() override {
        auto sol = std::make_unique<MSCFLPSolution>(inst_);
        phase1SelectFacilities(*sol);
        phase2AssignClients(*sol);
        sol->evaluate();
        return sol;
    }

private:
    const MSCFLPInstance& inst_;
    int  alpha_;
    int  beta_;
    std::mt19937 rng_;

    /**
     * @brief  Abre instalaciones usando una LRC de tamaño alpha sobre el
     *         criterio de coste fijo f_j, más una holgura adaptativa.
    */
    void phase1SelectFacilities(MSCFLPSolution& sol) {
        const int    m = inst_.getM();
        const int    n = inst_.getN();
        // Calcular demanda total
        double Dtotal = 0.0;
        for (int i = 0; i < n; ++i)
            Dtotal += inst_.getDemand(i);
        // Ordenar instalaciones por coste fijo ascendente
        std::vector<int> candidates(m);
        std::iota(candidates.begin(), candidates.end(), 0);
        std::sort(candidates.begin(), candidates.end(),
                  [&](int a, int b) {
                      return inst_.getFixedCost(a) < inst_.getFixedCost(b);
                  });
        double capacidadAcumulada = 0.0;
        // Abrir instalaciones hasta cubrir Dtotal, eligiendo al azar de LRC
        while (!candidates.empty() && capacidadAcumulada < Dtotal) {
            int lrcSize = std::min(alpha_, static_cast<int>(candidates.size()));
            std::uniform_int_distribution<int> dist(0, lrcSize - 1);
            int chosen = dist(rng_);
            int j = candidates[chosen];
            sol.openFacility(j);
            capacidadAcumulada += inst_.getCapacity(j);
            // Eliminar j de candidatos manteniendo el orden
            candidates.erase(candidates.begin() + chosen);
        }
        // Holgura adaptativa: más incompatibilidades → más instalaciones extra
        int slack = computeAdaptiveSlack();
        for (int t = 0; t < slack && !candidates.empty(); ++t) {
            int lrcSize = std::min(alpha_, static_cast<int>(candidates.size()));
            std::uniform_int_distribution<int> dist(0, lrcSize - 1);
            int chosen = dist(rng_);
            int j = candidates[chosen];
            sol.openFacility(j);
            candidates.erase(candidates.begin() + chosen);
        }
    }
    /**
     * @brief  Asigna la demanda de cada cliente usando una LRC de tamaño beta
     *         sobre el criterio de coste de transporte c_{ij}.
     */
    void phase2AssignClients(MSCFLPSolution& sol) {
        const int n = inst_.getN();
        const int m = inst_.getM();
        // Construir lista de instalaciones abiertas una sola vez
        std::vector<int> openFacilities;
        openFacilities.reserve(m);
        for (int j = 0; j < m; ++j)
            if (sol.isOpen(j))
                openFacilities.push_back(j);
        // Orden aleatorio de clientes 
        std::vector<int> clientOrder(n);
        std::iota(clientOrder.begin(), clientOrder.end(), 0);
        std::shuffle(clientOrder.begin(), clientOrder.end(), rng_);
        for (int i : clientOrder) {
            double demandaRestante = inst_.getDemand(i);
            while (demandaRestante > 1e-9) {
                // Filtrar instalaciones
                std::vector<std::pair<double, int>> feasibleList;
                feasibleList.reserve(openFacilities.size());

                for (int j : openFacilities) {
                    if (sol.getIncompCount(i, j) != 0) continue;   
                    double rj = sol.getResidualCap(j);
                    if (rj <= 1e-9) continue;
                    feasibleList.emplace_back(inst_.getTransportCost(i, j), j);
                }
                if (feasibleList.empty()) break;  
                // Ordenar por coste de transporte c_{ij} ascendente
                std::sort(feasibleList.begin(), feasibleList.end());
                // LRC = primeros min(beta, |feasibleList|) candidatos
                int lrcSize = std::min(beta_, static_cast<int>(feasibleList.size()));
                std::uniform_int_distribution<int> dist(0, lrcSize - 1);
                int chosen = dist(rng_);
                int    j  = feasibleList[chosen].second;
                double rj = sol.getResidualCap(j);
                double q  = std::min(demandaRestante, rj);
                sol.assignDemand(i, j, q);
                demandaRestante -= q;
            }
        }
    }

    /**
     * @brief  Calcula el número de instalaciones extra a abrir en fase 1
     *         en función de la densidad de incompatibilidades de la instancia.
     */
    int computeAdaptiveSlack() const {
        const int n = inst_.getN();
        if (n <= 1) return 5;

        double maxPairs = static_cast<double>(n) * (n - 1) / 2.0;
        double density  = static_cast<double>(inst_.getIncompatiblePairs().size())
                          / maxPairs;

        return static_cast<int>(std::round(5.0 + 10.0 * density));
    }

    void validateParams() const {
        if (alpha_ < 1)
            throw std::invalid_argument("GRASPConstructive: alpha debe ser >= 1");
        if (beta_ < 1)
            throw std::invalid_argument("GRASPConstructive: beta debe ser >= 1");
    }
};