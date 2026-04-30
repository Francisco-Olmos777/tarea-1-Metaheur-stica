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

int D;
int num_pistas = 3;
vector<Avion> aviones;
vector<vector<int>> tau;

double mejor_costo_global = 999999999;
vector<int> mejores_tiempos;
vector<int> mejores_pistas;

bool tiempo_agotado = false;
double tiempo_limite_segundos = 30.0;
high_resolution_clock::time_point tiempo_inicio;
long long nodos_explorados = 0;

double calcularCosto(int id_avion, int tiempo_asignado) {
    if (tiempo_asignado < aviones[id_avion].P)
        return (aviones[id_avion].P - tiempo_asignado) * aviones[id_avion].C;
    else if (tiempo_asignado > aviones[id_avion].P)
        return (tiempo_asignado - aviones[id_avion].P) * aviones[id_avion].C_prima;
    return 0.0;
}

bool propagarMFC(int indice_actual, int pista_asignada, int tiempo_asignado,
                 vector<vector<int>>& E_dinamico_pistas) {
    int id_k = aviones[indice_actual].id;
    for (int i = indice_actual + 1; i < D; i++) {
        int id_j = aviones[i].id;
        int nuevo_E = max(E_dinamico_pistas[pista_asignada][i],
                          tiempo_asignado + tau[id_k][id_j]);
        E_dinamico_pistas[pista_asignada][i] = nuevo_E;

        bool puede_aterrizar = false;
        for (int p = 1; p <= num_pistas; p++) {
            if (E_dinamico_pistas[p][i] <= aviones[i].L) {
                puede_aterrizar = true;
                break;
            }
        }
        if (!puede_aterrizar) return false;
    }
    return true;
}

void solveMFC(int indice_actual, vector<vector<int>>& E_dinamico_pistas,
              vector<int>& tiempos_asignados, vector<int>& pistas_asignadas,
              double costo_acumulado) {
    if (tiempo_agotado) return;
    auto ahora = high_resolution_clock::now();
    if (duration<double>(ahora - tiempo_inicio).count() >= tiempo_limite_segundos) {
        tiempo_agotado = true;
        return;
    }

    nodos_explorados++;

    // Caso base
    if (indice_actual == D) {
        mejor_costo_global = costo_acumulado;
        mejores_tiempos    = tiempos_asignados;
        mejores_pistas     = pistas_asignadas;
        return;
    }

    for (int p = 1; p <= num_pistas; p++) {
        pistas_asignadas[indice_actual] = p;

        // Mismo orden que BT y FC: P primero, luego P-1, P+1, P-2, P+2...
        // Solo incluir tiempos que siguen en el dominio (>= E_dinamico)
        int E_din = E_dinamico_pistas[p][indice_actual];
        int L_av  = aviones[indice_actual].L;
        int P_av  = aviones[indice_actual].P;
        vector<int> tiempos_posibles;
        if (P_av >= E_din && P_av <= L_av)
            tiempos_posibles.push_back(P_av);
        for (int d = 1; d <= max(P_av - E_din, L_av - P_av); d++) {
            if (P_av - d >= E_din && P_av - d <= L_av)
                tiempos_posibles.push_back(P_av - d);
            if (P_av + d >= E_din && P_av + d <= L_av)
                tiempos_posibles.push_back(P_av + d);
        }

        for (int t : tiempos_posibles) {
            vector<vector<int>> copia_E = E_dinamico_pistas;
            bool es_valido = propagarMFC(indice_actual, p, t, E_dinamico_pistas);

            if (es_valido) {
                tiempos_asignados[indice_actual] = t;
                solveMFC(indice_actual + 1, E_dinamico_pistas,
                         tiempos_asignados, pistas_asignadas,
                         costo_acumulado + calcularCosto(indice_actual, t));
            }
            E_dinamico_pistas = copia_E;
        }
        pistas_asignadas[indice_actual] = 0;
    }
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        cout << "Uso: ./mfc <archivo> <num_pistas> [limite_seg]" << endl;
        return 1;
    }
    num_pistas = stoi(argv[2]);
    if (argc >= 4) tiempo_limite_segundos = stod(argv[3]);

    ifstream archivo(argv[1]);
    if (!archivo.is_open()) {
        cerr << "Error: no se pudo abrir " << argv[1] << endl;
        return 1;
    }

    archivo >> D;
    aviones.resize(D);
    tau.assign(D, vector<int>(D, 0));
    for (int i = 0; i < D; i++) {
        aviones[i].id = i;
        archivo >> aviones[i].E >> aviones[i].P >> aviones[i].L
                >> aviones[i].C >> aviones[i].C_prima;
        for (int j = 0; j < D; j++) archivo >> tau[i][j];
    }
    archivo.close();

    // Ordenar por tiempo mas temprano (heuristica de variable)
    sort(aviones.begin(), aviones.end(), [](const Avion& a, const Avion& b) {
        return a.E == b.E ? a.L < b.L : a.E < b.E;
    });

    vector<vector<int>> E_din(num_pistas + 1, vector<int>(D));
    for (int p = 1; p <= num_pistas; p++)
        for (int i = 0; i < D; i++)
            E_din[p][i] = aviones[i].E;

    vector<int> tiempos(D, 0), pistas(D, 0);
    mejores_tiempos.resize(D, 0);
    mejores_pistas.resize(D, 0);

    tiempo_inicio = high_resolution_clock::now();
    solveMFC(0, E_din, tiempos, pistas, 0.0);
    double tiempo = duration<double>(high_resolution_clock::now() - tiempo_inicio).count();

    cout << "=== MFC ===" << endl;
    cout << "Estado          : " << (tiempo_agotado ? "TIEMPO AGOTADO (solucion parcial)" : "OPTIMO ENCONTRADO") << endl;
    cout << "Costo minimo    : " << mejor_costo_global  << endl;
    cout << "Nodos explorados: " << nodos_explorados     << endl;
    cout << "Tiempo          : " << tiempo << " s"       << endl;
    cout << "\nAsignacion optima:" << endl;
    for (int k = 0; k < D; k++)
        cout << "  Avion " << k
             << " -> Pista " << mejores_pistas[k]
             << "  T = "     << mejores_tiempos[k] << endl;

    return 0;
}