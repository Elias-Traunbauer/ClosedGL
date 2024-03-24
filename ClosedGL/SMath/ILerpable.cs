using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL.SMath
{
    public interface ILerpable<T> where T: new()
    {
        public T Lerp(T a, T b, float t);
    }
}
