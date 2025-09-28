# verificar_resultados.py
from collections import deque
import heapq

# --- Parámetros físicos para mostrar "Meta A (cm)" ---
B_CM    = 9.2     # punto de partida B
STEP_CM = 0.25    # ΔH por paso

def idx_to_pos(i: int) -> float:
    return round(B_CM + i * STEP_CM, 4)

# --- BFS (exhaustiva) ---
def bfs_line(meta_k, expand_first='right', limit=200_000):
    start=0
    if start==meta_k: return [0],0,1
    q=deque([start]); parent={start:None}; seen={start}
    opened=1; visited=0
    while q and visited<limit:
        cur=q.popleft(); visited+=1
        succ=[cur+1, cur-1] if expand_first=='right' else [cur-1, cur+1]
        for s in succ:
            if s not in seen:
                parent[s]=cur
                if s==meta_k:
                    path=[s]
                    while parent[path[-1]] is not None:
                        path.append(parent[path[-1]])
                    path.reverse()
                    return path, opened+1, visited+1
                q.append(s); seen.add(s); opened+=1
    return None, opened, visited

# --- A* (heurística) ---
def a_star_line(meta_k, limit=200_000):
    start=0
    if start==meta_k: return [0],0,1
    def h(i): return abs(meta_k-i)
    opened=0; visited=0
    pq=[(h(start),0,start)]
    parent={start:None}; g_cost={start:0}; closed=set()
    while pq and visited<limit:
        f,g,i=heapq.heappop(pq)
        if i in closed: continue
        closed.add(i); visited+=1
        if i==meta_k:
            path=[i]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            path.reverse(); return path, opened, visited
        for s in (i+1, i-1):
            ng=g+1
            if s not in g_cost or ng<g_cost[s]:
                g_cost[s]=ng; parent[s]=i
                heapq.heappush(pq,(ng+h(s),ng,s)); opened+=1
    return None, opened, visited

if __name__ == "__main__":
    CASES = [3, 6, -4, 12, -15]  # k en pasos

    print(f"Usando B={B_CM} cm, ΔH={STEP_CM} cm\n")
    print("| k (pasos) | Meta A (cm) | Método | Largo de ruta | Nodos abiertos | Nodos visitados |")
    print("|----------:|------------:|:------:|--------------:|---------------:|----------------:|")

    for k in CASES:
        # BFS
        ruta_b, ab_b, vi_b = bfs_line(k, expand_first='right')
        print(f"| {k:>10} | {idx_to_pos(k):>12} |  BFS   | {len(ruta_b):>13} | {ab_b:>15} | {vi_b:>16} |")

        # A*
        ruta_a, ab_a, vi_a = a_star_line(k)
        print(f"| {k:>10} | {idx_to_pos(k):>12} |  A*    | {len(ruta_a):>13} | {ab_a:>15} | {vi_a:>16} |")

    # --- (Opcional) Guardar CSV para pegar como imagen o adjuntar en Anexo ---
    # import csv
    # with open("resultados_busquedas_BFS_Astar.csv", "w", newline="", encoding="utf-8") as f:
    #     w = csv.writer(f)
    #     w.writerow(["k (pasos)","Meta A (cm)","Método","Largo de ruta","Nodos abiertos","Nodos visitados"])
    #     for k in CASES:
    #         ruta_b, ab_b, vi_b = bfs_line(k, expand_first='right'); w.writerow([k, idx_to_pos(k), "BFS", len(ruta_b), ab_b, vi_b])
    #         ruta_a, ab_a, vi_a = a_star_line(k);                     w.writerow([k, idx_to_pos(k), "A*",  len(ruta_a), ab_a, vi_a])
