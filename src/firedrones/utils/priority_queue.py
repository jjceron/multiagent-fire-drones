"""
Priority queue utility backed by Python's heapq.

Provides a simple min-heap with a tie-breaking counter so that
equal-priority items are ordered by insertion order (FIFO).
"""
from __future__ import annotations
import heapq
from typing import Any, Generic, TypeVar

T = TypeVar("T")


class PriorityQueue(Generic[T]):
    """
    A min-priority queue.

    Usage::

        pq = PriorityQueue()
        pq.push(item, priority)
        item = pq.pop()
        is_empty = pq.empty()
    """

    def __init__(self) -> None:
        self._heap: list[tuple[float, int, Any]] = []
        self._counter: int = 0

    def push(self, item: T, priority: float) -> None:
        """Push an item with the given priority (lower = higher urgency)."""
        heapq.heappush(self._heap, (priority, self._counter, item))
        self._counter += 1

    def pop(self) -> T:
        """Remove and return the item with the lowest priority value."""
        if self.empty():
            raise IndexError("pop from empty PriorityQueue")
        _, _, item = heapq.heappop(self._heap)
        return item

    def empty(self) -> bool:
        """Return True if the queue contains no items."""
        return len(self._heap) == 0

    def __len__(self) -> int:
        return len(self._heap)
