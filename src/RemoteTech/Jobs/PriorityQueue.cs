using System;
using Unity.Burst.CompilerServices;
using Unity.Collections;

namespace RemoteTech.Jobs;

internal struct PriorityQueue<T>(NativeList<T> items)
    where T : unmanaged, IComparable<T>
{
    NativeList<T> items = items;

    public readonly int Count => items.Length;
    public readonly bool IsEmpty => Count == 0;
    public readonly int Capacity => items.Capacity;

    public void Enqueue(T item)
    {
        items.Add(item);
        MoveUp(Count - 1);
    }

    [IgnoreWarning(1370)]
    public T Dequeue()
    {
        if (!TryDequeue(out var item))
            throw new InvalidOperationException("Attempted to call Dequeue() on an empty queue");

        return item;
    }

    public bool TryDequeue(out T item)
    {
        if (IsEmpty)
        {
            item = default;
            return false;
        }

        item = items[0];
        RemoveRootElement();
        return true;
    }

    [IgnoreWarning(1370)]
    public readonly T Peek()
    {
        if (!TryPeek(out var item))
            throw new InvalidOperationException("Attempted to call Peek() on an empty queue");
        return item;
    }

    public readonly bool TryPeek(out T item)
    {
        if (IsEmpty)
        {
            item = default;
            return false;
        }

        item = items[0];
        return true;
    }

    public void Clear() => items.Clear();

    private void Heapify()
    {
        // Heapify from the last non-leaf node down to the root
        for (int i = (Count - 2) / 2; i >= 0; --i)
            MoveDown(i);
    }

    private void MoveUp(int index)
    {
        var item = items[index];
        while (index != 0)
        {
            int parentIndex = GetParentindex(index);
            var parent = items[parentIndex];

            if (item.CompareTo(parent) < 0)
            {
                items[index] = parent;
                index = parentIndex;
            }
            else
            {
                break;
            }
        }

        items[index] = item;
    }

    private void MoveDown(int index)
    {
        var item = items[index];

        while (true)
        {
            var leftIndex = GetLeftChildIndex(index);
            var rightIndex = GetRightChildIndex(index);

            if (leftIndex >= Count)
                break;

            var left = items[leftIndex];
            var minIndex = leftIndex;
            var min = left;

            if (rightIndex < Count)
            {
                var right = items[rightIndex];

                if (right.CompareTo(left) < 0)
                {
                    minIndex = rightIndex;
                    min = right;
                }
            }

            if (item.CompareTo(min) < 0)
            {
                // Heap property is satisfied. We can insert the node here.
                break;
            }

            items[index] = min;
            index = minIndex;
        }

        items[index] = item;
    }

    private void RemoveRootElement() => RemoveAtIndex(0);

    private void RemoveAtIndex(int index)
    {
        items.RemoveAtSwapBack(index);
        if (index < Count)
            MoveDown(index);
    }

    static int GetParentindex(int index) => (index - 1) / 2;

    static int GetLeftChildIndex(int index) => index * 2 + 1;

    static int GetRightChildIndex(int index) => index * 2 + 2;
}
