import heapq
from mapeado import MAP_SIZE, occupancy_grid

def vecinos(pos):
    x, y = pos
    movimientos = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    result = []
    for dx, dy in movimientos:
        nx, ny = x + dx, y + dy
        if 0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE:
            if occupancy_grid[nx, ny] != 2:  # ✅ Solo obstáculo es bloqueante
                result.append((nx, ny))
    return result

def dijkstra(inicio, objetivo):
    heap = []
    heapq.heappush(heap, (0, inicio))
    distancias = {inicio: 0}
    predecesores = {inicio: None}

    while heap:
        costo_actual, nodo_actual = heapq.heappop(heap)

        if nodo_actual == objetivo:
            break

        for vecino in vecinos(nodo_actual):
            penalizacion = 1 if occupancy_grid[vecino[0], vecino[1]] == 1 else 2  # Penaliza desconocido
            nuevo_costo = costo_actual + penalizacion

            if vecino not in distancias or nuevo_costo < distancias[vecino]:
                distancias[vecino] = nuevo_costo
                heapq.heappush(heap, (nuevo_costo, vecino))
                predecesores[vecino] = nodo_actual

    camino = []
    nodo = objetivo
    while nodo is not None:
        camino.append(nodo)
        nodo = predecesores.get(nodo)
    camino.reverse()

    if camino and camino[0] == inicio:
        return camino
    else:
        return None
