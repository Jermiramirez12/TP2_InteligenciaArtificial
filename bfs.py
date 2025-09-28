# bfs.py — Búsqueda exhaustiva (BFS) en 1D con costo uniforme
# Modelo: estado i ∈ Z; posición física x(i) = B_CM + i * STEP_CM (en cm)

from collections import deque

# Parámetros físicos
B_CM    = 9.2      # cm (punto de partida B)
STEP_CM = 0.25     # cm por paso (ΔH)

def idx_to_pos(i: int) -> float:
    """Mapea el índice entero i a la posición física (cm)."""
    return round(B_CM + i * STEP_CM, 4)

def bfs_line(meta_k: int, expand_first: str = 'right', node_limit: int = 200_000):
    """
    BFS en una línea entera (0 es B). Meta exacta por índice k (A = B + k*STEP_CM).
    Retorna: (ruta_indices, nodos_abiertos, nodos_visitados)
    - expand_first: 'right' expande +1 luego -1 (o 'left' para lo inverso)
    - node_limit: tope de seguridad para evitar loops infinitos
    """
    start = 0
    if start == meta_k:
        return [0], 0, 1

    q = deque([start])
    parent = {start: None}
    seen = {start}
    opened = 1
    visited = 0

    while q and visited < node_limit:
        cur = q.popleft()
        visited += 1

        # Sucesores en el orden elegido
        succ = [cur + 1, cur - 1] if expand_first == 'right' else [cur - 1, cur + 1]

        for s in succ:
            if s not in seen:
                parent[s] = cur

                # ¿Se alcanzó la meta?
                if s == meta_k:
                    # reconstrucción de ruta
                    path = [s]
                    while parent[path[-1]] is not None:
                        path.append(parent[path[-1]])
                    path.reverse()
                    return path, opened + 1, visited + 1

                q.append(s)
                seen.add(s)
                opened += 1

    return None, opened, visited  # no encontrada dentro del límite

if __name__ == "__main__":
    # === Ejemplo reproducible del informe ===
    # Meta: k = +6  => A = B + 6 * ΔH = 9.2 + 6*0.25 = 10.7 cm
    k = 6
    A_cm = idx_to_pos(k)

    path_idx, opened, visited = bfs_line(meta_k=k, expand_first='right')
    path_cm = [idx_to_pos(i) for i in path_idx] if path_idx else None

    print(f"[BFS] B={B_CM} cm, ΔH={STEP_CM} cm, A={A_cm} cm (k={k})")
    print("Ruta (cm):", path_cm)
    print("Nodos abiertos:", opened, "Nodos visitados:", visited)

    # --- Prueba hacia la izquierda
    k2 = -4   # A = 9.2 - 4*0.25 = 8.2 cm
    A2_cm = idx_to_pos(k2)
    p2_idx, o2, v2 = bfs_line(meta_k=k2, expand_first='left')
    p2_cm = [idx_to_pos(i) for i in p2_idx] if p2_idx else None
    print(f"\n[BFS] B={B_CM} cm, ΔH={STEP_CM} cm, A={A2_cm} cm (k={k2})")
    print("Ruta (cm):", p2_cm)
    print("Nodos abiertos:", o2, "Nodos visitados:", v2)
