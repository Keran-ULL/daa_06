/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   Helper.h
 * @author Keran Miranda González
 * @date   2025-06-01
 * @brief  Funciones auxiliares de interfaz de usuario para main.cpp.
 */
#pragma once

#include <iostream>
#include <string>
#include <limits>
#include <filesystem>

namespace Helper {

enum class Algorithm {
    GREEDY  = 1,
    GRASP   = 2,
    QUIT    = 0
};

enum class Mode {
    SINGLE_INSTANCE = 1,
    BENCHMARK       = 2,
    BACK            = 0
};

enum class ImprovStrategy {
    BEST_IMPROVEMENT  = 1,
    FIRST_IMPROVEMENT = 2
};

/// Qué búsqueda(s) local(es) aplicar en la fase de mejora del GRASP
enum class LocalSearchChoice {
    SHIFT       = 1,   ///< Solo ShiftLS
    SWAP_CLI    = 2,   ///< Solo SwapClientesLS
    SWAP_INST   = 3,   ///< Solo SwapInstalacionesLS
    INCOMPAT    = 4,   ///< Solo IncompatElimLS
    ALL         = 5    ///< Las 4 en secuencia
};

inline void printBanner() {
    std::cout << "\n"
              << "╔══════════════════════════════════════════════════════╗\n"
              << "║   MS-CFLP-CI  —  DAA 2025-2026                      ║\n"
              << "║   Multi-Source Capacitated Facility Location         ║\n"
              << "║   Problem with Customer Incompatibilities            ║\n"
              << "╚══════════════════════════════════════════════════════╝\n\n";
}

/**
 * @brief  Lee un entero desde stdin. Repite hasta que la entrada sea válida
 *         y esté en [lo, hi].
 */
inline int readInt(const std::string& prompt, int lo, int hi) {
    int value;
    while (true) {
        std::cout << prompt;
        if (std::cin >> value && value >= lo && value <= hi) {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return value;
        }
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "  [!] Entrada no válida. Introduce un valor entre "
                  << lo << " y " << hi << ".\n";
    }
}

/**
 * @brief  Lee una cadena desde stdin (una línea completa).
 */
inline std::string readString(const std::string& prompt) {
    std::cout << prompt;
    std::string s;
    std::getline(std::cin, s);
    // Eliminar espacios iniciales/finales
    size_t start = s.find_first_not_of(" \t");
    size_t end   = s.find_last_not_of(" \t");
    if (start == std::string::npos) return "";
    return s.substr(start, end - start + 1);
}

inline Algorithm menuAlgorithm() {
    std::cout << "\n┌─ Selecciona algoritmo ──────────────────────────┐\n"
              << "│  1. Algoritmo Voraz                              │\n"
              << "│  2. GRASP (constructivo + búsqueda local)        │\n"
              << "│  0. Salir                                        │\n"
              << "└──────────────────────────────────────────────────┘\n";
    int opt = readInt("  Opción: ", 0, 2);
    return static_cast<Algorithm>(opt);
}

inline Mode menuMode(const std::string& algoName) {
    std::cout << "\n┌─ Modo de ejecución  [" << algoName << "] ─────────┐\n"
              << "│  1. Instancia individual                         │\n"
              << "│  2. Benchmark (wlp01-wlp08, tabla a fichero)     │\n"
              << "│  0. Volver                                       │\n"
              << "└──────────────────────────────────────────────────┘\n";
    int opt = readInt("  Opción: ", 0, 2);
    return static_cast<Mode>(opt);
}

inline std::string askInstancePath() {
    std::string path;
    while (true) {
        path = readString("  Ruta al fichero .dzn: ");
        if (path.empty()) {
            std::cout << "  [!] La ruta no puede estar vacía.\n";
            continue;
        }
        // Verificación básica de existencia
        if (!std::filesystem::exists(path)) {
            std::cout << "  [!] Fichero no encontrado: '" << path << "'\n"
                      << "      ¿Continuar igualmente? (s/n): ";
            char c; std::cin >> c;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            if (c == 's' || c == 'S') return path;
            continue;
        }
        return path;
    }
}

inline int askGreedySlack() {
    std::cout << "\n  Parámetro de holgura k (recomendado: 5): ";
    return readInt("", 0, 50);
}

struct GRASPParams {
    int              iterations;
    int              alpha;
    int              beta;
    unsigned int     seed;
    ImprovStrategy   strategy;
    LocalSearchChoice lsChoice;
};

inline LocalSearchChoice askLocalSearch() {
    std::cout << "\n  Búsqueda local en fase de mejora:\n"
              << "    1. ShiftLS              (reinserción de demanda)\n"
              << "    2. SwapClientesLS       (intercambio de clientes)\n"
              << "    3. SwapInstalacionesLS  (cierre/apertura instalación)\n"
              << "    4. IncompatElimLS       (eliminación incompatibilidades)\n"
              << "    5. Todas en secuencia\n";
    int opt = readInt("  Opción: ", 1, 5);
    return static_cast<LocalSearchChoice>(opt);
}

inline GRASPParams askGRASPParams() {
    GRASPParams p;
    std::cout << "\n  --- Parámetros GRASP ---\n";
    p.iterations = readInt("  Iteraciones (recomendado: 10-50): ", 1, 10000);
    p.alpha      = readInt("  Tamaño LRC fase 1 / alpha (2-5): ", 1, 20);
    p.beta       = readInt("  Tamaño LRC fase 2 / beta  (2-5): ", 1, 20);
    std::cout << "  Semilla RNG (0 = aleatoria): ";
    p.seed       = static_cast<unsigned int>(readInt("", 0, 999999));
    std::cout << "  Estrategia de mejora:\n"
              << "    1. Mejor mejora  (mejor calidad, más lento)\n"
              << "    2. Primera mejora (más rápido)\n";
    p.strategy   = static_cast<ImprovStrategy>(readInt("  Opción: ", 1, 2));
    p.lsChoice   = askLocalSearch();
    return p;
}

struct BenchmarkParams {
    std::string      instancesDir;
    std::string      outputFile;
    int              graspIter;
    int              graspRuns;
    LocalSearchChoice lsChoice;
};

inline BenchmarkParams askBenchmarkParams() {
    BenchmarkParams p;
    std::cout << "\n  --- Parámetros Benchmark ---\n";
    p.instancesDir = readString("  Directorio de instancias (ej: instances/): ");
    if (p.instancesDir.back() != '/' && p.instancesDir.back() != '\\')
        p.instancesDir += '/';
    p.outputFile = readString("  Fichero de salida (ej: resultados.txt): ");
    p.graspIter  = readInt("  Iteraciones GRASP por ejecución (recomendado: 10): ", 1, 1000);
    p.graspRuns  = readInt("  Ejecuciones por configuración LRC (recomendado: 3): ", 1, 20);
    p.lsChoice   = askLocalSearch();
    return p;
}

inline void printRunning(const std::string& algoName,
                          const std::string& instance) {
    std::cout << "\n  Ejecutando " << algoName
              << " sobre '" << instance << "'...\n";
}

inline void printSeparator() {
    std::cout << "  " << std::string(54, '-') << "\n";
}

inline void pressEnterToContinue() {
    std::cout << "\n  Pulsa Enter para continuar...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

} // namespace Helper