import heapq
import math
import geonamescache

def get_indian_cities(min_population=500000):
    gc = geonamescache.GeonamesCache()
    all_cities = gc.get_cities().values()
    india = [c for c in all_cities if c["countrycode"] == "IN" and c["population"] >= min_population]
    india.sort(key=lambda c: -c["population"])
    return {c["name"]: (float(c["latitude"]), float(c["longitude"])) for c in india}

def haversine(c1, c2):
    lat1, lon1 = math.radians(c1[0]), math.radians(c1[1])
    lat2, lon2 = math.radians(c2[0]), math.radians(c2[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return round(6371 * 2 * math.asin(math.sqrt(a)))

def build_graph(cities, max_neighbors=4):
    graph = {city: [] for city in cities}
    names = list(cities)
    for i, u in enumerate(names):
        distances = [(haversine(cities[u], cities[v]), v) for j, v in enumerate(names) if i != j]
        distances.sort()
        for dist, v in distances[:max_neighbors]:
            graph[u].append((v, dist))
            graph[v].append((u, dist))
    return graph

def dijkstra(graph, source):
    dist = {city: float("inf") for city in graph}
    prev = {city: None for city in graph}
    dist[source] = 0
    pq = [(0, source)]
    while pq:
        cost, u = heapq.heappop(pq)
        if cost > dist[u]:
            continue
        for v, w in graph[u]:
            if cost + w < dist[v]:
                dist[v] = cost + w
                prev[v] = u
                heapq.heappush(pq, (dist[v], v))
    return dist, prev

def get_path(prev, source, dest):
    path, node = [], dest
    while node:
        path.append(node)
        node = prev[node]
    path.reverse()
    return path if path[0] == source else []

def main():
    cities = get_indian_cities()
    print(f"Loaded {len(cities)} Indian cities (population >= 500k)\n")

    for i, name in enumerate(sorted(cities)):
        print(f"  {i+1:>3}. {name}")

    print()
    source = input("Source city: ").strip().title()
    dest   = input("Destination city: ").strip().title()

    if source not in cities or dest not in cities:
        print("City not found in list.")
        return

    graph = build_graph(cities)
    dist, prev = dijkstra(graph, source)

    d = dist[dest]
    if d == float("inf"):
        print("No path found.")
    else:
        path = get_path(prev, source, dest)
        print(f"\nShortest distance : {d} km")
        print(f"Route             : {' -> '.join(path)}")

if __name__ == "__main__":
    main()