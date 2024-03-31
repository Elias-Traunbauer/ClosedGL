using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL.Memory
{
    public unsafe class Pool<T> where T : unmanaged
    {
        private static Pool<T> instance;

        public static Pool<T> Instance
        {
            get
            {
                return instance ??= new Pool<T>();
            }
        }

        ~Pool()
        {
            Marshal.FreeHGlobal((IntPtr)Memory);
        }

        public static void Clear()
        {
            instance = null;
        }

        public static PoolMemoryBlock<T> Allocate(int size)
        {
            return Instance.Allocate1D(size);
        }

        public T* Memory { get; private set; }
        public int Capacity { get; private set; }
        public HashSet<PoolMemoryBlock<T>> poolMemoryBlocks = new HashSet<PoolMemoryBlock<T>>();

        public PoolMemoryBlock<T> Allocate1D(int size)
        {
            var index = FindFreeIndex(size);
            var memoryBlock = new PoolMemoryBlock<T>
            {
                Memory = (T*)Memory + index,
                startIndex = index,
                Size = size
            };
            return memoryBlock;
        }

        public int FindFreeIndex(int size)
        {
            var memoryBlocks = poolMemoryBlocks.OrderBy(x => x.startIndex).ToList();

            if (memoryBlocks.Count == 0)
            {
                // check if there is enough space at the end of the array
                if (Capacity < size)
                {
                    // allocate new memory
                    T* newMem = (T*)Marshal.AllocHGlobal(size * sizeof(T));
                    Buffer.MemoryCopy(Memory, newMem, Capacity * sizeof(T), Capacity * sizeof(T));
                    Capacity = size;
                    Marshal.FreeHGlobal((IntPtr)Memory);
                    Memory = newMem;
                }
                return 0;
            }

            int index = FindFirstLargeEnoughEmptyBlock(size);

            if (index == -1)
            {
                // allocate new memory
                var emtpySpaceAtEnd = Capacity - memoryBlocks.Last().startIndex - memoryBlocks.Last().Size;

                if (emtpySpaceAtEnd >= size)
                {
                    index = memoryBlocks.Last().startIndex + memoryBlocks.Last().Size;
                }
                else
                {
                    // allocate new memory
                    Marshal.ReAllocHGlobal((IntPtr)Memory, (Capacity + size - emtpySpaceAtEnd) * sizeof(T));
                    Capacity = Capacity + size - emtpySpaceAtEnd;
                }
            }

            return index;
        }

        public int FindFirstLargeEnoughEmptyBlock(int size)
        {
            var memoryBlocks = poolMemoryBlocks.OrderBy(x => x.startIndex).ToList();

            for (int i = 0; i < memoryBlocks.Count; i++)
            {
                if (i == 0)
                {
                    if (memoryBlocks[i].startIndex >= size)
                    {
                        return 0;
                    }
                }
                else
                {
                    if (memoryBlocks[i].startIndex - memoryBlocks[i - 1].startIndex >= size)
                    {
                        return memoryBlocks[i - 1].startIndex + memoryBlocks[i - 1].Size;
                    }
                }
            }

            return -1;
        }
    }

    public unsafe class PoolMemoryBlock<T> where T : unmanaged
    {
        public T* Memory { get; set; }

        public int startIndex;
        public int Size { get; set; }

        public T this[int index]
        {
            get
            {
                if (index < 0)
                {
                    throw new IndexOutOfRangeException();
                }
                if (index >= Size)
                {
                    throw new IndexOutOfRangeException();
                }
                return Memory[index];
            }
            set
            {
                if (index < 0)
                {
                    throw new IndexOutOfRangeException();
                }
                if (index >= Size)
                {
                    throw new IndexOutOfRangeException();
                }
                Memory[index] = value;
            }
        }

        public void Free()
        {
            Pool<T>.Instance.poolMemoryBlocks.Remove(this);
        }

        public void Clear()
        {
            for (int i = 0; i < Size; i++)
            {
                Memory[i] = default;
            }
        }

        public void Fill(T value)
        {
            for (int i = 0; i < Size; i++)
            {
                Memory[i] = value;
            }
        }

        public void CopyFrom(T* source, int length)
        {
            Buffer.MemoryCopy(source, Memory, length * sizeof(T), length * sizeof(T));
        }

        public void CopyTo(T* destination, int length)
        {
            Buffer.MemoryCopy(Memory, destination, length * sizeof(T), length * sizeof(T));
        }

        public void CopyFrom(T[] source, int length)
        {
            fixed (T* sourcePtr = source)
            {
                Buffer.MemoryCopy(sourcePtr, Memory, length * sizeof(T), length * sizeof(T));
            }
        }

        public void CopyTo(T[] destination, int length)
        {
            fixed (T* destinationPtr = destination)
            {
                Buffer.MemoryCopy(Memory, destinationPtr, length * sizeof(T), length * sizeof(T));
            }
        }

        public void CopyFrom(T[] source, int sourceIndex, int length)
        {
            fixed (T* sourcePtr = source)
            {
                Buffer.MemoryCopy(sourcePtr + sourceIndex, Memory, length * sizeof(T), length * sizeof(T));
            }
        }
    }
}
