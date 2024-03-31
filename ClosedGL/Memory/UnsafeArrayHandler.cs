using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL.Memory
{
    public class UnsafeArrayHandler
    {
        public static unsafe TDest[] FlattenAndCastArrays<TSrc, TDest, TObj>(IEnumerable<TObj> source, Func<TObj, TSrc[]> selector) where TSrc : unmanaged where TDest : unmanaged
        {
            int totalLength = source.Sum(x => selector(x).Length);
            TSrc[] result = new TSrc[totalLength];
            TDest[] resultCasted = new TDest[totalLength];
            int currentIndex = 0;
            foreach (TObj ob in source)
            {
                TSrc[] array = selector(ob);
                Array.Copy(array, 0, result, currentIndex, array.Length);
                currentIndex += array.Length;
            }
            fixed (TSrc* p = result)
            {
                fixed (TDest* q = resultCasted)
                {
                    Buffer.MemoryCopy(p, q, totalLength * sizeof(TSrc), totalLength * sizeof(TDest));
                }
            }
            return resultCasted;
        }

        public static unsafe TDest[] FlattenAndCastArraysBeta<TSrc, TDest, TObj>(IEnumerable<TObj> source, Func<TObj, TSrc[]> selector) where TSrc : unmanaged where TDest : unmanaged
        {
            int totalLength = source.Sum(x => selector(x).Length);
            TDest[] resultCasted = new TDest[totalLength];
            fixed (TDest* destination = resultCasted)
            {
                int currentIndex = 0;
                foreach (TObj ob in source)
                {
                    TSrc[] array = selector(ob);
                    fixed (TSrc* arrayPtr = array)
                    {
                        Buffer.MemoryCopy(arrayPtr, destination + currentIndex, array.Length * sizeof(TSrc), array.Length * sizeof(TDest));
                    }
                    currentIndex += array.Length;
                }
            }
            return resultCasted;
        }

        public static unsafe TDest[] CastArrayBeta<TSrc, TDest>(TSrc[] source) where TSrc : unmanaged where TDest : unmanaged
        {
            int sourceByteLength = source.Length * sizeof(TSrc);
            int totalLength = source.Length;
            int destLength = sourceByteLength / sizeof(TDest);
            TDest[] resultCasted = new TDest[destLength];
            fixed (TDest* destination = resultCasted)
            {
                fixed (TSrc* arrayPtr = source)
                {
                    Buffer.MemoryCopy(arrayPtr, destination, resultCasted.Length * sizeof(TSrc), resultCasted.Length * sizeof(TDest));
                }
            }
            return resultCasted;
        }

        public static unsafe TSrc[] FlattenArrays<TSrc, TObj>(IEnumerable<TObj> source, Func<TObj, TSrc[]> selector) where TSrc : unmanaged
        {
            int totalLength = source.Sum(x => selector(x).Length);
            TSrc[] result = new TSrc[totalLength];
            int currentIndex = 0;
            foreach (TObj ob in source)
            {
                TSrc[] array = selector(ob);
                Array.Copy(array, 0, result, currentIndex, array.Length);
                currentIndex += array.Length;
            }
            return result;
        }

        public static unsafe TDest[] ExtractFieldAndCastArray<TSrc, TDest, TObj>(IEnumerable<TObj> source, Func<TObj, TSrc> selector) where TSrc : unmanaged where TDest : unmanaged
        {
            int totalLength = source.Count();
            TDest[] result = new TDest[totalLength];
            int currentIndex = 0;
            foreach (TObj ob in source)
            {
                TSrc field = selector(ob);
                TSrc* p = &field;
                TDest* q = (TDest*)p;
                result[currentIndex] = *q;
                currentIndex += 1;
            }
            return result;
        }
    }
}
