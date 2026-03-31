/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * Práctica 5: MS-CFLP-CI
 * 
 * @file   Algorithm.h
 * @brief  Jerarquía de clases abstractas para los algoritmos del MS-CFLP-CI.
 * @author Keran Miranda González
 * @version 1.0
 * @date 2025-06-01
 */

#pragma once

#include "Instance.h"
#include "Solution.h"

#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>

/**
 * @class  Algorithm
 * @brief  Contrato genérico para cualquier algoritmo de optimización.
 *
 * Gestiona la referencia a la instancia del problema y la temporización
 * de la ejecución.  Las subclases implementan run() con su lógica concreta.
 */
class Algorithm {
public:

    /**
     * @param  instance  Instancia del problema a resolver.
     *                   Debe estar cargada antes de llamar a run().
     */
    explicit Algorithm(const Instance& instance) : instance_(instance), elapsedMs_(0.0) {}

    virtual ~Algorithm() = default;

    Algorithm(const Algorithm&)            = delete;
    Algorithm& operator=(const Algorithm&) = delete;
    Algorithm(Algorithm&&)                 = default;
    Algorithm& operator=(Algorithm&&)      = default;

    /**
     * @brief  Ejecuta el algoritmo completo.
     * @return Mejor solución encontrada (puede no ser óptima).
     *
     * Internamente mide el tiempo transcurrido, que queda disponible
     * a través de getElapsedMs() tras la llamada.
     */
    std::unique_ptr<Solution> run() {
      auto start = std::chrono::steady_clock::now();
      auto result = solve();
      auto end   = std::chrono::steady_clock::now();
      elapsedMs_ = std::chrono::duration<double, std::milli>(end - start).count();
      return result;
    }

    /**
     * @brief  Nombre legible del algoritmo (para tablas de resultados).
     */
    virtual std::string getName() const = 0;

    /** @brief Tiempo de ejecución de la última llamada a run() en ms. */
    double getElapsedMs() const { return elapsedMs_; }

    /** @brief Acceso de solo lectura a la instancia del problema. */
    const Instance& getInstance() const { return instance_; }

protected:

    /**
     * @brief  Lógica interna del algoritmo.  Solo debe llamarse desde run().
     * @return Mejor solución construida o mejorada.
     */
    virtual std::unique_ptr<Solution> solve() = 0;

    const Instance& instance_;
    double elapsedMs_;
};

/**
 * @class  ConstructiveAlgorithm
 * @brief  Algoritmo que construye una solución inicial desde cero.
 *
 * Subclases concretas: Greedy, GRASPConstructive.
 *
 * La fase constructiva no recibe ninguna solución de entrada;
 * crea y devuelve una solución nueva (posiblemente no óptima).
 */
class ConstructiveAlgorithm : public Algorithm {
public:
    explicit ConstructiveAlgorithm(const Instance& instance) : Algorithm(instance) {}
    virtual ~ConstructiveAlgorithm() = default;

    /**
     * @brief  Parámetro opcional para controlar la aleatoriedad de la
     *         construcción (p.ej. tamaño de la LRC en GRASP).
     *         El valor 0 implica construcción determinista (voraz puro).
     */
    virtual void setRandomizationParam(int param) { (void)param; }

protected:
    // solve() se hereda de Algorithm y llama a buildSolution() internamente.
    // Las subclases simplemente implementan solve() construyendo y devolviendo
    // una nueva solución, o bien pueden sobrescribir buildSolution() si prefieren
    // separar la construcción de la evaluación.
};

/**
 * @class  LocalSearch
 * @brief  Búsqueda local greedy que itera sobre un entorno de movimientos.
 *
 * Subclases concretas: ShiftLS, SwapClientsLS, SwapFacilitiesLS,
 *                      IncompatElimLS.
 *
 * Una búsqueda local recibe una solución de entrada (pasada por referencia)
 * y la mejora in-place hasta alcanzar un óptimo local respecto al entorno
 * que define.  También devuelve la solución mejorada como puntero por
 * coherencia con el contrato de Algorithm.
 */
class LocalSearch : public Algorithm {
public:
    explicit LocalSearch(const Instance& instance) : Algorithm(instance), maxIterations_(std::numeric_limits<int>::max()) {}
    virtual ~LocalSearch() = default;

    /**
     * @brief  Mejora una solución existente mediante la búsqueda local.
     * @param  solution  Solución de entrada; se modifica in-place.
     * @return true  si se encontró al menos una mejora, false si ya era
     *               un óptimo local respecto a este entorno.
     *
     * Este es el método principal de las búsquedas locales.
     * run() delega en él con una solución vacía (no debe usarse así;
     * use siempre improve()).
     */
    virtual bool improve(Solution& solution) = 0;

    /**
     * @brief  Aplica un único movimiento de mejora (primera mejora o
     *         mejor mejora según la implementación).
     * @param  solution  Solución sobre la que se aplica el movimiento.
     * @return true  si el movimiento mejoró el coste.
     */
    virtual bool applyBestMove(Solution& solution) = 0;

    /**
     * @brief  Limita el número máximo de iteraciones de la búsqueda.
     *         Por defecto es ilimitado (INT_MAX).
     */
    void setMaxIterations(int maxIter) { maxIterations_ = maxIter; }

    int getMaxIterations() const { return maxIterations_; }

protected:
    int maxIterations_;

    /**
     * solve() de una LocalSearch no tiene sentido sin una solución de entrada.
     * Lanzamos una excepción para forzar el uso correcto de improve().
     */
    std::unique_ptr<Solution> solve() override {
        throw std::logic_error(
            "LocalSearch::solve() — use improve(solution) en vez de run()."
        );
    }
};

/**
 * @class  Metaheuristic
 * @brief  Marco genérico para metaheurísticas (GRASP, GVNS, RVND, etc.).
 *
 * Una metaheurística orquesta un algoritmo constructivo y una o varias
 * búsquedas locales durante múltiples iteraciones, manteniendo y
 * devolviendo la mejor solución encontrada.
 *
 * Subclases concretas: GRASP, GVNS_RL, RVND.
 */
class Metaheuristic : public Algorithm {
public:
    /**
     * @param  instance        Instancia del problema.
     * @param  maxIterations   Número máximo de iteraciones del bucle principal.
     * @param  seed            Semilla para el generador de números aleatorios.
     *                         Usar 0 para semilla basada en tiempo.
     */
    Metaheuristic(const Instance& instance, int maxIterations, unsigned int seed = 0)
        : Algorithm(instance)
        , maxIterations_(maxIterations)
        , seed_(seed == 0
                    ? static_cast<unsigned int>(
                          std::chrono::steady_clock::now()
                              .time_since_epoch().count())
                    : seed_)
    {}

    virtual ~Metaheuristic() = default;

    void setMaxIterations(int maxIter)  { maxIterations_ = maxIter; }
    int  getMaxIterations() const       { return maxIterations_; }
    void setSeed(unsigned int seed)     { seed_ = seed; }
    unsigned int getSeed() const        { return seed_; }

    /**
     * @brief  Devuelve el número de iteraciones ejecutadas en la última
     *         llamada a run().
     */
    int getIterationsRun() const { return iterationsRun_; }

protected:
    int maxIterations_;
    unsigned int seed_;
    int iterationsRun_ = 0;
};
