# astar.py — Búsqueda heurística (A*) en 1D con costo uniforme
# Modelo: estado i ∈ Z; posición física x(i) = B_CM + i * STEP_CM (cm)

import heapq

# Parámetros físicos (distintos a los de tu compañero)
B_CM    = 9.2      # cm (punto de partida B)
STEP_CM = 0.25     # cm por paso (ΔH)

def idx_to_pos(i: int) -> float:
    """Mapea el índice entero i a la posición física (cm)."""
    return round(B_CM + i * STEP_CM, 4)

def a_star_line(meta_k: int, node_limit: int = 200_000):
    """
    A* en línea (0 es B). Heurística: h(i) = |meta_k - i|  (admisible/consistente).
    Retorna: (ruta_indices, nodos_abiertos, nodos_visitados)
    """
    start = 0
    if start == meta_k:
        return [0], 0, 1

    def h(i: int) -> int:
        return abs(meta_k - i)

    opened = 0
    visited = 0
    # (f, g, i): f = g + h
    pq = [(h(start), 0, start)]
    parent = {start: None}
    g_cost = {start: 0}
    closed = set()

    while pq and visited < node_limit:
        f, g, i = heapq.heappop(pq)
        if i in closed:
            continue
        closed.add(i)
        visited += 1

        if i == meta_k:
            # reconstruir ruta
            path = [i]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            path.reverse()
            return path, opened, visited

        for s in (i + 1, i - 1):  # sucesores: derecha/izquierda
            ng = g + 1
            if s not in g_cost or ng < g_cost[s]:
                g_cost[s] = ng
                parent[s] = i
                fs = ng + h(s)
                heapq.heappush(pq, (fs, ng, s))
                opened += 1

    return None, opened, visited  # no encontrada dentro del límite

if __name__ == "__main__":
    # === Ejemplo reproducible del informe ===
    # Meta: k = +6  => A = B + 6 * ΔH = 9.2 + 6*0.25 = 10.7 cm
    k = 6
    A_cm = idx_to_pos(k)

    path_idx, opened, visited = a_star_line(meta_k=k)
    path_cm = [idx_to_pos(i) for i in path_idx] if path_idx else None

    print(f"[A*] B={B_CM} cm, ΔH={STEP_CM} cm, A={A_cm} cm (k={k})")
    print("Ruta (cm):", path_cm)
    print("Nodos abiertos:", opened, "Nodos visitados:", visited)

    # --- Pruebas opcionales ---
    # k2 = -4   # A = 9.2 - 4*0.25 = 8.2 cm
    # A2_cm = idx_to_pos(k2)
    # p2_idx, o2, v2 = a_star_line(meta_k=k2)
    # p2_cm = [idx_to_pos(i) for i in p2_idx] if p2_idx else None
    # print(f"\n[A*] B={B_CM} cm, ΔH={STEP_CM} cm, A={A2_cm} cm (k={k2})")
    # print("Ruta (cm):", p2_cm)
    # print("Nodos abiertos:", o2, "Nodos visitados:", v2)
