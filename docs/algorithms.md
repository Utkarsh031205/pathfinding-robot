# Algorithmic Foundations: Autonomous Robot Navigation

This document details the theoretical and mathematical foundations of the three graph search algorithms implemented in the Autonomous Robot Navigation Simulator: **A\* Search**, **Dijkstra's Algorithm**, and **Greedy Best-First Search**.

---

## 1. Graph Formulation of the 2D Warehouse Floor

The warehouse grid is modeled as an undirected, weighted graph $G = (V, E)$:
- **Vertices ($V$):** Every traversable grid cell $(r, c)$ in the grid bounds $[0, R-1] \times [0, C-1]$.
- **Edges ($E$):** Four-directional orthogonal movements:
  $$\{(u, v) \in V \times V \mid |r_u - r_v| + |c_u - c_v| = 1\}$$
  Diagonal movement is strictly forbidden.
- **Cost Model:** Destination-cell cost convention:
  $$\text{cost}(u \to v) = c(v)$$
  where $c(v)$ represents the positive traversal cost of entering cell $v$ (e.g., standard concrete floor = $1.0$, express lane = $0.5$, heavy rough floor = $5.0$, congested spill = $10.0$).
  Entering the start position incurs an initial entry cost of $0.0$.

---

## 2. A* Search

### 2.1 Evaluation Function
A* explores the graph using the composite evaluation function:
$$f(n) = g(n) + h(n)$$
where:
- $g(n)$: The exact accumulated cost along the best-known path from start to node $n$.
- $h(n)$: An admissible heuristic estimating the cheapest remaining cost from $n$ to the goal.
- $f(n)$: The estimated total cost of the cheapest path constrained to pass through $n$.

### 2.2 Heuristic Admissibility & Consistency
For a 4-directional grid with uniform cost ($c = 1.0$), the standard **Manhattan Distance** ($L_1$ norm) is used:
$$h_{\text{manhattan}}(n, \text{goal}) = |r_n - r_{\text{goal}}| + |c_n - c_{\text{goal}}|$$

However, when the environment contains **nonuniform weighted terrain** where traversal costs vary, using unscaled Manhattan distance would violate admissibility if any cell has cost less than $1.0$ (e.g., express conveyor lane at $0.5$). 

To preserve **mathematical admissibility** ($h(n) \le h^*(n)$ for all $n$):
$$h(n) = (|r_n - r_{\text{goal}}| + |c_n - c_{\text{goal}}|) \times c_{\min}$$
where $c_{\min} = \min_{v \in V_{\text{passable}}} c(v) > 0$.

Because every single step on the grid must cost at least $c_{\min}$, any valid route from $n$ to the goal must have cost:
$$h^*(n) \ge \text{steps} \times c_{\min} \ge h_{\text{manhattan}}(n, \text{goal}) \times c_{\min} = h(n)$$
This guarantees that A* never overestimates remaining distance and will **always return an optimal-cost path**.

### 2.3 Tie-Breaking
When multiple frontier nodes have identical $f$-scores, our implementation uses $h(n)$ as a secondary tie-breaker. Choosing nodes with smaller $h(n)$ steers exploration toward the goal, reducing the volume of explored nodes without sacrificing optimality.

### 2.4 Complexity
- **Time Complexity:** $O((|V| + |E|) \log |V|)$ in the worst case with a binary heap, reducing toward $O(d)$ when the heuristic is highly directed.
- **Space Complexity:** $O(|V|)$ to store the open priority queue and closed set.

---

## 3. Dijkstra's Algorithm

### 3.1 Uniform-Cost Exploration
Dijkstra's algorithm is an optimal uniform-cost search. It sets:
$$f(n) = g(n) \quad (\text{equivalent to } A^* \text{ with } h(n) = 0)$$

The priority queue orders nodes purely by their accumulated path cost from the start. Nodes are expanded outward in concentric contours of equal traversal cost.

### 3.2 Optimality Guarantee
Because all edge traversal costs $c(v) > 0$ are strictly positive, once a node $u$ is popped from the min-heap priority queue, no future discovery can reveal a shorter path to $u$. 
Dijkstra serves as the **unbiased, optimal-cost ground truth baseline** against which all other algorithms are compared.

### 3.3 Complexity
- **Time Complexity:** $O((|V| + |E|) \log |V|)$ with a binary min-heap.
- **Space Complexity:** $O(|V|)$ for distance scores, closed set, and priority queue.

---

## 4. Greedy Best-First Search

### 4.1 Heuristic Priority
Greedy Best-First Search prioritizes nodes solely on estimated geometric proximity to the goal:
$$\text{priority}(n) = h(n) = |r_n - r_{\text{goal}}| + |c_n - c_{\text{goal}}|$$

It completely ignores $g(n)$, the accumulated traversal cost.

### 4.2 Behavior & Suboptimality
- **Speed:** When the obstacle landscape is open, Greedy can sprint directly toward the goal, expanding very few nodes.
- **Failure Mode / Suboptimality:** On weighted terrain, Greedy is blind to cost. For example, if a direct path cuts straight through a high-cost hazardous spill (cost = 10.0 per cell) while an express bypass (cost = 0.5 per cell) loops slightly around, Greedy will choose the expensive swamp simply because each cell is geometrically closer to the goal. The resulting path can be orders of magnitude more expensive than the optimal route found by Dijkstra or A*.
- **Local Minima / Dead Ends:** Greedy can get pulled into concave obstacle pockets, requiring extensive backtracking.

---

## 5. Comparative Metric Summary

| Metric | Dijkstra's Algorithm | A* Search | Greedy Best-First Search |
| :--- | :--- | :--- | :--- |
| **Priority Function** | $g(n)$ | $g(n) + h(n)$ | $h(n)$ |
| **Optimal Path Cost?** | **Guaranteed** | **Guaranteed** (with admissible $h$) | **Not Guaranteed** |
| **Explored Node Volume** | High (explores all directions) | Moderate / Low (focused toward goal) | Low (direct) to High (in traps) |
| **Terrain Awareness** | Full | Full | None (only in final report) |
| **Admissibility Required?** | N/A ($h = 0$) | Yes ($c_{\min}$-scaled Manhattan) | No (used as primary priority) |
