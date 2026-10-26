import {
  AlgorithmResult,
  AlgorithmType,
  BenchmarkRecord,
  MultiScenarioSummary,
} from '../types/simulator';
import { WarehouseGrid } from './grid';
import { runAlgorithm } from './algorithms';

/**
 * Runs all three algorithms (A*, Dijkstra, Greedy) on an immutable snapshot of the grid.
 */
export function runComparativeBenchmark(
  grid: WarehouseGrid,
  scenarioName: string = 'Custom Scenario'
): {
  astar: AlgorithmResult;
  dijkstra: AlgorithmResult;
  greedy: AlgorithmResult;
  records: BenchmarkRecord[];
} {
  const algorithms: AlgorithmType[] = ['astar', 'dijkstra', 'greedy'];
  const results: Record<AlgorithmType, AlgorithmResult> = {} as any;
  const records: BenchmarkRecord[] = [];
  const now = new Date().toISOString();

  // Run on independent clones so no mutation occurs
  for (const algo of algorithms) {
    const gridSnapshot = grid.clone();
    // Warm-up iteration to reduce JIT compilation bias
    runAlgorithm(algo, gridSnapshot.clone(), false);

    // Timed iteration
    const result = runAlgorithm(algo, gridSnapshot, true);
    results[algo] = result;

    records.push({
      id: `${algo}-${Date.now()}-${Math.random().toString(36).substring(2, 6)}`,
      scenarioName,
      algorithm: algo,
      found: result.found,
      pathLength: result.pathLength,
      totalCost: result.totalCost,
      expandedNodes: result.expandedNodes,
      discoveredNodes: result.discoveredNodes,
      generatedNodes: result.generatedNodes,
      executionTimeMs: result.executionTimeMs,
      replanCount: 0,
      gridRows: grid.rows,
      gridCols: grid.cols,
      obstacleDensity: 0,
      timestamp: now,
    });
  }

  return {
    astar: results['astar'],
    dijkstra: results['dijkstra'],
    greedy: results['greedy'],
    records,
  };
}

export interface MultiRunConfig {
  numScenarios: number;
  rows: number;
  cols: number;
  obstacleDensity: number;
  seed: number;
  includeTerrain: boolean;
}

/**
 * Executes repeated multi-scenario experiments across multiple random seeds
 * and returns detailed records and aggregated statistics.
 */
export function runBatchExperiments(config: MultiRunConfig): {
  records: BenchmarkRecord[];
  summaries: MultiScenarioSummary[];
} {
  const records: BenchmarkRecord[] = [];
  const algorithms: AlgorithmType[] = ['astar', 'dijkstra', 'greedy'];

  for (let s = 0; s < config.numScenarios; s++) {
    const scenarioSeed = config.seed + s * 137;
    const testGrid = new WarehouseGrid(config.rows, config.cols);
    testGrid.start = { r: 1, c: 1 };
    testGrid.goal = { r: config.rows - 2, c: config.cols - 2 };

    testGrid.generateRandomObstacles(config.obstacleDensity, scenarioSeed);
    if (config.includeTerrain) {
      testGrid.generateRandomTerrain(scenarioSeed + 17);
    }

    const scenarioName = `Scenario #${s + 1} (Seed ${scenarioSeed})`;
    const comparison = runComparativeBenchmark(testGrid, scenarioName);

    for (const rec of comparison.records) {
      rec.obstacleDensity = config.obstacleDensity;
      records.push(rec);
    }
  }

  // Compute aggregated summaries for each algorithm
  const summaries: MultiScenarioSummary[] = algorithms.map((algo) => {
    const algoRecords = records.filter((r) => r.algorithm === algo);
    const runs = algoRecords.length;
    const successfulRuns = algoRecords.filter((r) => r.found);
    const successRate = runs > 0 ? (successfulRuns.length / runs) * 100 : 0;

    const times = algoRecords.map((r) => r.executionTimeMs).sort((a, b) => a - b);
    const avgTimeMs = runs > 0 ? times.reduce((a, b) => a + b, 0) / runs : 0;
    const medianTimeMs =
      runs > 0
        ? runs % 2 === 1
          ? times[Math.floor(runs / 2)]
          : (times[runs / 2 - 1] + times[runs / 2]) / 2
        : 0;

    const avgCost =
      successfulRuns.length > 0
        ? successfulRuns.reduce((a, b) => a + b.totalCost, 0) / successfulRuns.length
        : 0;

    const avgExpanded =
      runs > 0 ? algoRecords.reduce((a, b) => a + b.expandedNodes, 0) / runs : 0;

    const avgPathLength =
      successfulRuns.length > 0
        ? successfulRuns.reduce((a, b) => a + b.pathLength, 0) / successfulRuns.length
        : 0;

    return {
      algorithm: algo,
      runs,
      successRate: Number(successRate.toFixed(1)),
      avgTimeMs: Number(avgTimeMs.toFixed(3)),
      medianTimeMs: Number(medianTimeMs.toFixed(3)),
      avgCost: Number(avgCost.toFixed(2)),
      avgExpanded: Number(avgExpanded.toFixed(1)),
      avgPathLength: Number(avgPathLength.toFixed(1)),
    };
  });

  return { records, summaries };
}

/**
 * Converts benchmark records to CSV format for download.
 */
export function exportRecordsToCSV(records: BenchmarkRecord[]): string {
  const headers = [
    'Scenario',
    'Algorithm',
    'Path Found',
    'Path Length (steps)',
    'Total Cost',
    'Expanded Nodes',
    'Discovered Nodes',
    'Generated Nodes',
    'Execution Time (ms)',
    'Grid Dimensions',
    'Obstacle Density',
    'Timestamp',
  ];

  const rows = records.map((r) => [
    `"${r.scenarioName}"`,
    r.algorithm.toUpperCase(),
    r.found ? 'TRUE' : 'FALSE',
    r.pathLength,
    r.totalCost,
    r.expandedNodes,
    r.discoveredNodes,
    r.generatedNodes,
    r.executionTimeMs,
    `"${r.gridRows}x${r.gridCols}"`,
    r.obstacleDensity,
    `"${r.timestamp}"`,
  ]);

  return [headers.join(','), ...rows.map((row) => row.join(','))].join('\n');
}
