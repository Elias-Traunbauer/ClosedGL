using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL
{
    public struct Vector3I
    {
        public int X, Y, Z;

        public Vector3I(int x, int y, int z)
        {
            X = x; Y = y; Z = z;
        }

        public static Vector3I Forward = new Vector3I(0, 0, 1);
        public static Vector3I Up = new Vector3I(0, 1, 0);
        public static Vector3I Right = new Vector3I(1, 0, 0);

        public static Vector3I Backward = -Forward;
        public static Vector3I Down = -Up;
        public static Vector3I Left = -Right;

        public static Vector3I operator +(Vector3I a, Vector3I b)
        {
            return new Vector3I(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }

        public static Vector3I operator -(Vector3I a, Vector3I b)
        {
            return new Vector3I(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }

        public static Vector3I operator -(Vector3I a)
        {
            return new Vector3I(-a.X, -a.Y, -a.Z);
        }

        public static Vector3I operator *(Vector3I a, int b)
        {
            return new Vector3I(a.X * b, a.Y * b, a.Z * b);
        }

        public static Vector3I operator *(int b, Vector3I a)
        {
            return new Vector3I(a.X * b, a.Y * b, a.Z * b);
        }

        public static Vector3I operator /(Vector3I a, int b)
        {
            return new Vector3I(a.X / b, a.Y / b, a.Z / b);
        }

        public static Vector3I operator /(int b, Vector3I a)
        {
            return new Vector3I(a.X / b, a.Y / b, a.Z / b);
        }

        public static bool operator ==(Vector3I a, Vector3I b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
        }

        public static bool operator !=(Vector3I a, Vector3I b)
        {
            return !(a == b);
        }

        public static double Dot(Vector3I a, Vector3I b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static Vector3I Cross(Vector3I a, Vector3I b)
        {
            return new Vector3I(
                               a.Y * b.Z - a.Z * b.Y,
                                              a.Z * b.X - a.X * b.Z,
                                                             a.X * b.Y - a.Y * b.Z);
        }

        public int Length()
        {
            return (int)Math.Sqrt(X * X + Y * Y + Z * Z);
        }

        public Vector3I Normalize()
        {
            return this / Length();
        }

        public override string ToString()
        {
            return $"({X}, {Y}, {Z})";
        }
    }
}
