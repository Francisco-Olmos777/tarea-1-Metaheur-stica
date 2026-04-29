#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <chrono>

using namespace std;
using namespace std::chrono;

struct Avion {
    int id;
    int E, P, L;
    double C, C_prima;
};

// Variables Globales
int D; 
int num_pistas = 3; 
vector<Avion> aviones; //para? -->forma de interpretar los datos del .txt y mantener el ID original para imprimir resultados
vector<vector<int>> tau;  // Matriz de separacion entre aviones i y j

// Estado de la solucion
double mejor_costo_global = 999999999;
vector<int> mejores_tiempos;
vector<int> mejores_pistas;

// Control de tiempo
bool tiempo_agotado = false;
double tiempo_limite_segundos = 300.0; // 5 minutos
high_resolution_clock::time_point tiempo_inicio;

// Funcion que calcula el costo
double calcularCosto(int id_avion, int tiempo_asignado) {
    if (tiempo_asignado < aviones[id_avion].P) {
        return (aviones[id_avion].P - tiempo_asignado) * aviones[id_avion].C;
    } else if (tiempo_asignado > aviones[id_avion].P) {
        return (tiempo_asignado - aviones[id_avion].P) * aviones[id_avion].C_prima;
    }
    return 0.0;
}

// EL VERDADERO MFC: Actualiza los límites inferiores (Bounds) de las pistas futuras
bool propagarMFC(int indice_actual, int pista_asignada, int tiempo_asignado, vector<vector<int>>& E_dinamico_pistas) {
    int id_k = aviones[indice_actual].id;

    // Propagamos hacia los aviones en el FUTURO
    for (int i = indice_actual + 1; i < D; i++) {
        int id_j = aviones[i].id;

        // Actualizamos el límite E(tiempo temprano) SOLO para la pista que acabamos de ocupar
        int nuevo_E = max(E_dinamico_pistas[pista_asignada][i], tiempo_asignado + tau[id_k][id_j]);
        E_dinamico_pistas[pista_asignada][i] = nuevo_E;

        // REGLA MFC: Verificamos si el avion i se quedó sin opciones en TODAS las pistas
        bool puede_aterrizar = false;
        for(int p = 1; p <= num_pistas; p++) {
            if (E_dinamico_pistas[p][i] <= aviones[i].L) {
                puede_aterrizar = true; // Aún sobrevive en al menos una pista
                break;
            }
        }

        if (!puede_aterrizar) return false; // MFC detectó colapso. Rama muerta.
    }
    return true;
}

// LA RECURSION EXACTA (Backtracking + Minimal Forward Checking)
void solveMFC(int indice_actual, vector<vector<int>>& E_dinamico_pistas, vector<int>& tiempos_asignados, vector<int>& pistas_asignadas, double costo_acumulado) {
    
    //tiempo limite
    if (tiempo_agotado) return; 
    auto tiempo_actual = high_resolution_clock::now();
    duration<double> tiempo_transcurrido = duration_cast<duration<double>>(tiempo_actual - tiempo_inicio);
    if (tiempo_transcurrido.count() >= tiempo_limite_segundos) {
        tiempo_agotado = true; 
        return; 
    }

    // Poda por costo
    if (costo_acumulado >= mejor_costo_global) return;
 
    // Caso Base
    if (indice_actual == D) {
        mejor_costo_global = costo_acumulado;
        mejores_tiempos = tiempos_asignados;
        mejores_pistas = pistas_asignadas;

        // Imprimir tiempo en vivo cada vez que encontramos una mejor solucion 
        auto tiempo_actual = high_resolution_clock::now();
        double segs = duration_cast<duration<double>>(tiempo_actual - tiempo_inicio).count();
        cout << "T=" << segs << "s -> Costo: $" << mejor_costo_global << endl;

        return;
    }

    // NODO ACTUAL: Iterar sobre pistas
    for (int p = 1; p <= num_pistas; p++) {
        pistas_asignadas[indice_actual] = p;

        // Heuristica de Valor: Recopilamos tiempos validos DESDE el E de esta pista especifica
        vector<int> tiempos_posibles;
        for (int t = E_dinamico_pistas[p][indice_actual]; t <= aviones[indice_actual].L; t++) {
            tiempos_posibles.push_back(t);
        }

        // Ordenamos tiempos probando primero el tiempo Preferente (P) para evitar multas y luego por costo creciente si no se puede evitar la multa
        sort(tiempos_posibles.begin(), tiempos_posibles.end(), [&](int t1, int t2) {
            return calcularCosto(indice_actual, t1) < calcularCosto(indice_actual, t2);
        });

        // Iterar sobre los tiempos posibles
        for (int t : tiempos_posibles) {
            
            // 1. Snapshot del MFC
            vector<vector<int>> copia_E = E_dinamico_pistas;

            // 2. Propagar MFC (Mirar al futuro)
            bool es_valido = propagarMFC(indice_actual, p, t, E_dinamico_pistas);

            // 3. Si el MFC dice que el futuro es viable, avanzamos
            if (es_valido) {
                double costo_paso = calcularCosto(indice_actual, t);
                tiempos_asignados[indice_actual] = t;
                
                solveMFC(indice_actual + 1, E_dinamico_pistas, tiempos_asignados, pistas_asignadas, costo_acumulado + costo_paso);
            }

            // 4. Backtracking: Restaurar el radar MFC si la rama anterior no fue buena, asi pasa al siguiente tiempo posible
            E_dinamico_pistas = copia_E;
        }
        
        pistas_asignadas[indice_actual] = 0; // Backtracking de pista
    }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------

void mostrarSolucion(double costo_final, const vector<int>& tiempos_finales, const vector<int>& pistas_finales, double tiempo_ejecucion) {
    cout << "\n==================================================\n";
    cout << "              RESULTADOS DE LA ASIGNACION         \n";
    cout << "==================================================\n";
    
    // Verificamos si realmente encontró una solución (asumiendo que tu "infinito" es un número muy grande)
    if (costo_final >= 1e9) { 
        cout << "No se encontro ninguna solucion factible en el tiempo dado." << endl;
    } else {
        cout << "Tiempo de ejecucion : " << tiempo_ejecucion << " segundos" << endl;
        cout << "Costo Total Optimo  : $" << costo_final << endl;
        cout << "--------------------------------------------------\n";
        cout << "Detalle por Avion:\n";
        
        for (size_t i = 0; i < tiempos_finales.size(); i++) {
            cout << "  Avion " << (aviones[i].id + 1) 
                 << "\t-> Pista: " << pistas_finales[i] 
                 << "\t| Minuto de aterrizaje: " << tiempos_finales[i] << endl;
        }
    }
    cout << "==================================================\n";
}

bool inicializarEntorno(const string& nombre_archivo, vector<vector<int>>& E_dinamico_pistas, vector<int>& tiempos_asignados, vector<int>& pistas_asignadas) {
    // 1. Cargar el archivo
    ifstream archivo(nombre_archivo);
    if (!archivo.is_open()) {
        cerr << "Error: No se pudo abrir el archivo: " << nombre_archivo << endl;
        return false; 
    }

    archivo >> D;
    aviones.resize(D);
    tau.assign(D, vector<int>(D, 0)); 

    for (int i = 0; i < D; i++) {
        aviones[i].id = i;
        archivo >> aviones[i].E >> aviones[i].P >> aviones[i].L >> aviones[i].C >> aviones[i].C_prima;
        for (int j = 0; j < D; j++) {
            archivo >> tau[i][j];
        }
    }
    archivo.close();

    // 2. Heurística de Variable: Ordenar por Tiempo Más Temprano (E)
    sort(aviones.begin(), aviones.end(), [](const Avion &a, const Avion &b) {
        if (a.E == b.E) return a.L < b.L; 
        return a.E < b.E; 
    });

    // 3. Inicializar tamaños de las matrices y vectores locales
    E_dinamico_pistas.assign(num_pistas + 1, vector<int>(D));
    for (int p = 1; p <= num_pistas; p++) {
        for (int i = 0; i < D; i++) {
            E_dinamico_pistas[p][i] = aviones[i].E;
        }
    }

    tiempos_asignados.assign(D, 0);
    pistas_asignadas.assign(D, 0);

    return true; // Todo se inicializó correctamente
}

//------------------------------------------------------------------------------------------------------------------------------------------------------

int main() {
    // Declaración de estructuras locales
    vector<vector<int>> E_dinamico_pistas;
    vector<int> tiempos_asignados;
    vector<int> pistas_asignadas;

    // Inicializar todo (Archivo, Heurística y Matrices)
    if (!inicializarEntorno("case1.txt", E_dinamico_pistas, tiempos_asignados, pistas_asignadas)) {
        return 1; // Falla si el archivo no existe
    }

    // Iniciar el reloj
    cout << "\nIniciando busqueda MFC (Limite: " << tiempo_limite_segundos / 60.0 << " min)..." << endl;
    tiempo_inicio = high_resolution_clock::now();

    // Ejecutar el algoritmo
    solveMFC(0, E_dinamico_pistas, tiempos_asignados, pistas_asignadas, 0.0);

    // Detener reloj y mostrar resultados
    auto tiempo_fin = high_resolution_clock::now();
    duration<double> tiempo_total = duration_cast<duration<double>>(tiempo_fin - tiempo_inicio);

    //Mostrar resultados finales
    mostrarSolucion(mejor_costo_global, mejores_tiempos, mejores_pistas, tiempo_total.count());

    return 0;
}



/*grafico de :

Gráfico de Explosión Combinatoria (Tiempo vs. N)
    Eje X: Número de aviones (Casos: 15, 20, 44, 100).
    Eje Y: Tiempo de ejecución en segundos.

Histograma de Uso de Pistas (El "Efecto Goloso")

    Eje X: Pistas (Pista 1, Pista 2, Pista 3).
    Eje Y: Cantidad de aviones asignados a esa pista.

Tabla Resumen de Ejecución
    Instancia (Case 1, 2, 3, 4)
    Tamaño ($N$) (15, 20, 44, 100)
    Mejor Costo Encontrado * Tiempo de Ejecución (X seg / "Time Limit")
    Estado de la Solución (Óptimo Demostrado / Subóptimo por Time Limit)

*/