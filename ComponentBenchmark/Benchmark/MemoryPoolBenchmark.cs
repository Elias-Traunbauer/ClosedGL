using ClosedGL.Memory;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ComponentBenchmark.Benchmark
{
    public class MemoryPoolBenchmark
    {
        [Benchmark]
        public void NativeAllocations()
        {
            for (int i = 0; i < 1000; i++) 
            {
                var array = new int[1000];
            }
        }

        [Benchmark]
        public void PoolAllocations()
        {
            for (int i = 0; i < 1000; i++)
            {
                var array = Pool<int>.Allocate(1000);
                //array.Free();
            }
        }
    }
}
