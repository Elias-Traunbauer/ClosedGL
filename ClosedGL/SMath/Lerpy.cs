using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL.SMath
{
    public class Lerpy<T>(T[] points, float[] values) where T : ILerpable<T>, new()
    {
        private T[] points = points;
        private float[] values = values;

        public T GetValue(float x)
        {
            x = x % 1;
            float currentFactor = 0;
            float previousFactor = 0;
            T previousPoint = points[0];
            for (int i = 1; i < points.Length; i++)
            {
                currentFactor = values[i];

                if (x < currentFactor)
                {
                    float endFactor = values[i];
                    float startFactor = previousFactor;
                    float t = (x - startFactor) / (endFactor - startFactor);
                    return T.Lerp(previousPoint, points[i], t);
                }

                previousFactor = currentFactor;
                previousPoint = points[i];
            }
            throw new ArithmeticException("Value out of range");
            return default;
        }
    }

    public class LerpyLength<T> where T : ILerpable<T>, ILengthable<T>, new()
    {
        private T[] points;
        private float[] values;
        
        public LerpyLength(T[] points)
        {
            this.points = points;
            this.values = new float[points.Length];
            float curr = 0;
            T prev = points[0];
            values[0] = 0;
            for (int i = 1; i < points.Length; i++)
            {
                curr += prev.To(points[i]).Length();
                values[i] = curr;
                prev = points[i];
            }
            // Normalize
            for (int i = 0; i < values.Length; i++)
            {
                values[i] /= curr;
            }
            values[^1] = 1;
        }

        public T GetValue(float x)
        {
            x = x % 1;
            float currentFactor = 0;
            float previousFactor = 0;
            T previousPoint = points[0];
            for (int i = 1; i < points.Length; i++)
            {
                currentFactor = values[i];

                if (x < currentFactor)
                {
                    float endFactor = values[i];
                    float startFactor = previousFactor;
                    float t = (x - startFactor) / (endFactor - startFactor);
                    return T.Lerp(previousPoint, points[i], t);
                }

                previousFactor = currentFactor;
                previousPoint = points[i];
            }
            throw new ArithmeticException("Value out of range");
            return default;
        }
    }
}
