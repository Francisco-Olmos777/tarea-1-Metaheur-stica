#include <bits/stdc++.h>
using namespace std;

// ============================================================
// ESTRUCTURAS (mapean directo al modelo CSP)
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
    int n;                       // numero de aviones |K|
    int r;                       // numero de pistas  |R|
    vector<Avion> aviones;
    vector<vector<int>> sep;     // sep[i][j] = tau_ij
};

struct Asignacion {
    int T = -1;   // tiempo de aterrizaje (T_k)
    int r = -1;   // pista asignada       (r_k)
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

// ============================================================
// COSTO DE UN AVION DADO SU TIEMPO ASIGNADO
// FO: C_k * max(0, P_k - T_k) + C'_k * max(0, T_k - P_k)
// ============================================================

double costo_avion(int k, int T) {
    int P = inst.aviones[k].P;
    if (T < P)
        return inst.aviones[k].c  * (P - T);
    else if (T > P)
        return inst.aviones[k].cp * (T - P);
    else
        return 0.0;  // T == P, aterrizo justo en tiempo preferente
}
// ============================================================
// COSTO TOTAL DE LA SOLUCION COMPLETA
// ============================================================

double calcula_costo_total(const vector<Asignacion>& asig) {
    double costo = 0;
    for (int k = 0; k < inst.n; k++)
        costo += costo_avion(k, asig[k].T);
    return costo;
}

// ============================================================
// FACTIBILIDAD
// Verifica si asignar tiempo T en pista r al avion k
// respeta tau con TODOS los aviones ya asignados en esa pista
// Restriccion del modelo:
//   r_i = r_k ^ T_i < T  =>  T  - T_i >= sep[i][k]
//   r_i = r_k ^ T_i > T  =>  T_i - T  >= sep[k][i]
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
// BACKTRACKING CRONOLOGICO
// Procesa aviones en orden fijo 0..n-1
// Ordena el dominio probando P_k primero para activar poda rapido
// Se detiene si se supera el limite de tiempo
// ============================================================

void backtracking(int k) {
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

    // --- Orden del dominio: P primero, luego P-1, P+1, P-2, P+2 ... ---
    vector<int> orden_T;
    orden_T.push_back(av.P);
    for (int d = 1; d <= max(av.P - av.E, av.L - av.P); d++) {
        if (av.P - d >= av.E) orden_T.push_back(av.P - d);
        if (av.P + d <= av.L) orden_T.push_back(av.P + d);
    }

    for (int r = 1; r <= inst.r; r++) {
        for (int T : orden_T) {
            if (tiempo_agotado) return;
            if (es_factible(k, T, r)) {
                asignacion_actual[k] = {T, r};
                backtracking(k + 1);
                asignacion_actual[k] = {-1, -1};
            }
        }
    }
}

// ============================================================
// LECTOR DE INSTANCIA (formato OR-Library ALP)
//   n
//   E_i P_i L_i c_i cp_i
//   sep[i][0] ... sep[i][n-1]
// Numero de pistas se pasa como argumento al ejecutable
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
// Permite podar mas ramas desde el inicio del arbol
// ============================================================

void heuristica_variables() {
    sort(inst.aviones.begin(), inst.aviones.end(),
         [](const Avion& a, const Avion& b) {
             return a.E == b.E ? a.L < b.L : a.E < b.E;
         });
}

// ============================================================
// MAIN
// Uso: ./backtracking <archivo_instancia> <num_pistas> [limite_seg]
// ============================================================

int main(int argc, char* argv[]) {
    if (argc < 3) {
        cout << "Uso: ./backtracking <archivo_instancia> <num_pistas> [limite_seg]" << endl;
        return 1;
    }

    int num_pistas = stoi(argv[2]);
    if (argc >= 4) limite_segundos = stod(argv[3]);

    inst = leer_instancia(argv[1], num_pistas);
    asignacion_actual.resize(inst.n, {-1, -1});
    mejor_asignacion.resize(inst.n, {-1, -1});
    mejor_costo = 1e18;

    heuristica_variables();  // ordenar aviones por E antes de buscar
    t_inicio = time(0);
    backtracking(0);
    double tiempo = difftime(time(0), t_inicio);

    cout << "=== BACKTRACKING CRONOLOGICO ===" << endl;
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
