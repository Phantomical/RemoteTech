using System;
using System.Collections;
using System.Collections.Generic;

namespace RemoteTech.Jobs;

internal struct IntRange(int start, int length) : IEnumerable<int>
{
    public int start = start;
    public int length = length;

    public readonly Enumerator GetEnumerator() => new(this);

    readonly IEnumerator<int> IEnumerable<int>.GetEnumerator() => GetEnumerator();

    readonly IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    public struct Enumerator(IntRange range) : IEnumerator<int>
    {
        int index = range.start - 1;
        readonly int end = range.start + range.length;

        public readonly int Current => index;
        readonly object IEnumerator.Current => Current;

        public bool MoveNext()
        {
            index += 1;
            return index < end;
        }

        public void Dispose() { }

        void IEnumerator.Reset() => throw new NotSupportedException();
    }
}
