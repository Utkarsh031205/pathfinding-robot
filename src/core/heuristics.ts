import { Coordinate } from '../types/simulator';

export type HeuristicType = 'manhattan' | 'scaled_manhattan' | 'euclidean';

/**
 * Standard Manhattan Distance: |r1 - r2| + |c1 - c2|
 * Admissible for uniform grid with cost = 1.0.
 */
export function manhattanDistance(a: Coordinate, b: Coordinate): number {
  return Math.abs(a.r - b.r) + Math.abs(a.c - b.c);
}

/**
 * Scaled Manhattan Distance: Manhattan(a, b) * minCost.
 * Admissible for nonuniform terrain where cell costs >= minCost.
 * Guarantees h(n) <= true remaining shortest distance cost.
 */
export function scaledManhattanDistance(
  a: Coordinate,
  b: Coordinate,
  minCost: number
): number {
  return (Math.abs(a.r - b.r) + Math.abs(a.c - b.c)) * minCost;
}

/**
 * Euclidean Distance: sqrt((r1 - r2)^2 + (c1 - c2)^2)
 */
export function euclideanDistance(a: Coordinate, b: Coordinate): number {
  const dr = a.r - b.r;
  const dc = a.c - b.c;
  return Math.sqrt(dr * dr + dc * dc);
}
