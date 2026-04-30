#include <bits/stdc++.h>
using namespace std;

// ============================================================
// ESTRUCTURAS (identicas al modelo del BT)
// ============================================================

struct Avion {
    int id;
    int E;       // tiempo minimo    (E_k)
    int P;       // tiempo preferente (P_k)
    int L;       // tiempo maximo    (L_k)
    double c;    // costo si T < P   (C_k)
    double cp;   // costo si T >= P  (C'_k)
};

struct Instancia {
    int n;
    int r;
    vector<Avion> aviones;
    vector<vector<int>> sep;     // sep[i][j] = tau_ij
};

struct Asignacion {
    int T = -1;
    int r = -1;
};

// ============================================================
// VARIABLES GLOBALES
// ============================================================

Instancia inst;
vector<Asignacion> asignacion_actual;
vector<Asignacion> mejor_asignacion;
double mejor_costo;
long long nodos_explorados = 0;
bool tiempo_agotado = false;
time_t t_inicio;
double limite_segundos = 60.0;

// dom[j][r] = set de tiempos validos para avion j en pista r
// Se poda con FC y se restaura al hacer backtrack
vector<vector<set<int>>> dom;

// ============================================================
// COSTO DE UN AVION
// ============================================================

double costo_avion(int k, int T) {
    int P = inst.aviones[k].P;
    if (T < P)       return inst.aviones[k].c  * (P - T);
    else if (T > P)  return inst.aviones[k].cp * (T - P);
    else             return 0.0;
}

// ============================================================
// COSTO TOTAL
// ============================================================

double calcula_costo_total(const vector<Asignacion>& asig) {
    double costo = 0;
    for (int k = 0; k < inst.n; k++)
        costo += costo_avion(k, asig[k].T);
    return costo;
}

// ============================================================
// FACTIBILIDAD (igual que BT)
// ============================================================

bool es_factible(int k, int T, int r) {
    for (int i = 0; i < k; i++) {
        if (asignacion_actual[i].r != r) continue;
        int Ti = asignacion_actual[i].T;
        if (Ti == T)                             return false;
        if (Ti < T && T  - Ti < inst.sep[i][k]) return false;
        if (Ti > T && Ti - T  < inst.sep[k][i]) return false;
    }
    return true;
}

// ============================================================
// FORWARD CHECK
// Despues de asignar avion k con (T_k, r_k), mira hacia adelante:
// Para cada avion futuro j > k, elimina de dom[j][r_k] los tiempos
// que violarian la separacion con (T_k, r_k).
//
// Guarda lo eliminado en 'eliminados' para poder restaurarlo.
// Retorna false si algun avion futuro queda sin ningun tiempo
// valido en ninguna pista (dominio vacio = callejon sin salida).
// ============================================================

bool forward_check(int k, int T_k, int r_k,
                   vector<tuple<int,int,int>>& eliminados) {
    for (int j = k + 1; j < inst.n; j++) {

        // Recopilar tiempos a eliminar de dom[j][r_k]
        vector<int> a_eliminar;
        for (int Tj : dom[j][r_k]) {
            if (Tj == T_k) {
                a_eliminar.push_back(Tj);
            } else if (T_k < Tj && Tj - T_k < inst.sep[k][j]) {
                a_eliminar.push_back(Tj);
            } else if (T_k > Tj && T_k - Tj < inst.sep[j][k]) {
                a_eliminar.push_back(Tj);
            }
        }

        // Eliminar y guardar para restaurar despues
        for (int Tj : a_eliminar) {
            dom[j][r_k].erase(Tj);
            eliminados.push_back({j, Tj, r_k});
        }

        // Verificar si avion j todavia tiene algun tiempo valido
        // en alguna pista (si no, es callejon sin salida)
        bool tiene_dominio = false;
        for (int r = 1; r <= inst.r; r++) {
            if (!dom[j][r].empty()) {
                tiene_dominio = true;
                break;
            }
        }
        if (!tiene_dominio) return false;
    }
    return true;
}

// ============================================================
// RESTAURAR DOMINIOS
// Deshace lo que elimino forward_check al hacer backtrack
// ============================================================

void restaurar(const vector<tuple<int,int,int>>& eliminados) {
    for (auto& [j, T, r] : eliminados)
        dom[j][r].insert(T);
}

// ============================================================
// FORWARD CHECKING
// Igual que BT pero:
// - Itera sobre dom[k][r] en vez del rango completo [E,L]
// - Llama a forward_check antes de recursar
// - Restaura dominios al hacer backtrack
// ============================================================

void fc(int k) {
    // Chequear limite de tiempo cada 10000 nodos
    if (nodos_explorados % 10000 == 0) {
        if (difftime(time(0), t_inicio) >= limite_segundos) {
            tiempo_agotado = true;
            return;
        }
    }
    if (tiempo_agotado) return;

    nodos_explorados++;

    // --- Condicion de termino ---
    if (k == inst.n) {
        double costo = calcula_costo_total(asignacion_actual);
        if (costo < mejor_costo) {
            mejor_costo      = costo;
            mejor_asignacion = asignacion_actual;
        }
        return;
    }

    Avion& av = inst.aviones[k];

    // --- Orden del dominio: P primero, luego P-1, P+1 ... ---
    vector<int> orden_T;
    orden_T.push_back(av.P);
    for (int d = 1; d <= max(av.P - av.E, av.L - av.P); d++) {
        if (av.P - d >= av.E) orden_T.push_back(av.P - d);
        if (av.P + d <= av.L) orden_T.push_back(av.P + d);
    }

    for (int r = 1; r <= inst.r; r++) {
        for (int T : orden_T) {
            if (tiempo_agotado) return;

            // Solo probar si T sigue en el dominio de k en pista r
            if (dom[k][r].find(T) == dom[k][r].end()) continue;

            if (es_factible(k, T, r)) {
                asignacion_actual[k] = {T, r};

                // Mirar al futuro: podar dominios
                vector<tuple<int,int,int>> eliminados;
                bool factible = forward_check(k, T, r, eliminados);

                if (factible) {
                    fc(k + 1);
                }

                // Restaurar dominios (backtrack)
                restaurar(eliminados);
                asignacion_actual[k] = {-1, -1};
            }
        }
    }
}

// ============================================================
// INICIALIZAR DOMINIOS
// dom[j][r] empieza con todos los tiempos [E_j, L_j]
// ============================================================

void inicializar_dominios() {
    dom.resize(inst.n, vector<set<int>>(inst.r + 1));
    for (int j = 0; j < inst.n; j++)
        for (int r = 1; r <= inst.r; r++)
            for (int T = inst.aviones[j].E; T <= inst.aviones[j].L; T++)
                dom[j][r].insert(T);
}

// ============================================================
// LECTOR DE INSTANCIA (mismo formato que BT)
// ============================================================

Instancia leer_instancia(const string& archivo, int num_pistas) {
    Instancia inst;
    ifstream f(archivo);
    if (!f.is_open()) {
        cerr << "Error: no se pudo abrir " << archivo << endl;
        exit(1);
    }
    f >> inst.n;
    inst.r = num_pistas;
    inst.aviones.resize(inst.n);
    inst.sep.assign(inst.n, vector<int>(inst.n));
    for (int i = 0; i < inst.n; i++) {
        inst.aviones[i].id = i;
        f >> inst.aviones[i].E
          >> inst.aviones[i].P
          >> inst.aviones[i].L
          >> inst.aviones[i].c
          >> inst.aviones[i].cp;
        for (int j = 0; j < inst.n; j++)
            f >> inst.sep[i][j];
    }
    return inst;
}

// ============================================================
// HEURISTICA DE SELECCION DE VARIABLES
// Ordena aviones por E creciente (el mas restringido primero)
// Si empatan en E, desempata por L creciente (ventana mas estrecha)
// ============================================================

void heuristica_variables() {
    sort(inst.aviones.begin(), inst.aviones.end(),
         [](const Avion& a, const Avion& b) {
             return a.E == b.E ? a.L < b.L : a.E < b.E;
         });
}

// ============================================================
// MAIN
// Uso: ./fc <archivo_instancia> <num_pistas> [limite_seg]
// ============================================================

int main(int argc, char* argv[]) {
    if (argc < 3) {
        cout << "Uso: ./fc <archivo_instancia> <num_pistas> [limite_seg]" << endl;
        return 1;
    }

    int num_pistas = stoi(argv[2]);
    if (argc >= 4) limite_segundos = stod(argv[3]);

    inst = leer_instancia(argv[1], num_pistas);
    heuristica_variables();   // ordenar aviones por E antes de buscar
    inicializar_dominios();

    asignacion_actual.resize(inst.n, {-1, -1});
    mejor_asignacion.resize(inst.n, {-1, -1});
    mejor_costo = 1e18;

    t_inicio = time(0);
    fc(0);
    double tiempo = difftime(time(0), t_inicio);

    cout << "=== FORWARD CHECKING ===" << endl;
    cout << "Estado          : " << (tiempo_agotado ? "TIEMPO AGOTADO (solucion parcial)" : "OPTIMO ENCONTRADO") << endl;
    cout << "Costo minimo    : " << mejor_costo      << endl;
    cout << "Nodos explorados: " << nodos_explorados  << endl;
    cout << "Tiempo          : " << tiempo << " s"    << endl;
    cout << "\nAsignacion optima:" << endl;
    for (int k = 0; k < inst.n; k++) {
        cout << "  Avion " << k
             << " -> Pista " << mejor_asignacion[k].r
             << "  T = "     << mejor_asignacion[k].T
             << "  costo = " << costo_avion(k, mejor_asignacion[k].T)
             << endl;
    }

    return 0;
}
