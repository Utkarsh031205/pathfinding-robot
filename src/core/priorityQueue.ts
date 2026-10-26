/**
 * High-performance Binary Min-Heap Priority Queue for graph search algorithms.
 * Handles stale items and priority updates gracefully with counter tie-breaking.
 */
export interface PQElement<T> {
  item: T;
  priority: number;
  insertOrder: number;
  secondaryPriority?: number; // e.g. h(n) tie breaker in A*
}

export class PriorityQueue<T> {
  private heap: PQElement<T>[] = [];
  private counter: number = 0;

  constructor() {}

  public get size(): number {
    return this.heap.length;
  }

  public isEmpty(): boolean {
    return this.heap.length === 0;
  }

  public push(item: T, priority: number, secondaryPriority?: number): void {
    const node: PQElement<T> = {
      item,
      priority,
      insertOrder: this.counter++,
      secondaryPriority,
    };
    this.heap.push(node);
    this.bubbleUp(this.heap.length - 1);
  }

  public pop(): T | undefined {
    if (this.isEmpty()) return undefined;
    const top = this.heap[0];
    const bottom = this.heap.pop()!;
    if (this.heap.length > 0) {
      this.heap[0] = bottom;
      this.bubbleDown(0);
    }
    return top.item;
  }

  public clear(): void {
    this.heap = [];
    this.counter = 0;
  }

  private compare(a: PQElement<T>, b: PQElement<T>): number {
    if (a.priority !== b.priority) {
      return a.priority - b.priority;
    }
    if (a.secondaryPriority !== undefined && b.secondaryPriority !== undefined) {
      if (a.secondaryPriority !== b.secondaryPriority) {
        return a.secondaryPriority - b.secondaryPriority;
      }
    }
    return a.insertOrder - b.insertOrder;
  }

  private bubbleUp(index: number): void {
    while (index > 0) {
      const parent = Math.floor((index - 1) / 2);
      if (this.compare(this.heap[index], this.heap[parent]) < 0) {
        [this.heap[index], this.heap[parent]] = [this.heap[parent], this.heap[index]];
        index = parent;
      } else {
        break;
      }
    }
  }

  private bubbleDown(index: number): void {
    const length = this.heap.length;
    while (true) {
      let smallest = index;
      const left = 2 * index + 1;
      const right = 2 * index + 2;

      if (left < length && this.compare(this.heap[left], this.heap[smallest]) < 0) {
        smallest = left;
      }
      if (right < length && this.compare(this.heap[right], this.heap[smallest]) < 0) {
        smallest = right;
      }

      if (smallest !== index) {
        [this.heap[index], this.heap[smallest]] = [this.heap[smallest], this.heap[index]];
        index = smallest;
      } else {
        break;
      }
    }
  }
}
