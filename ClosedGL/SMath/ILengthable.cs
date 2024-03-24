using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL.SMath
{
    public interface ILengthable<T>
    {
        public float Length();
        public T To(T to);
    }
}
