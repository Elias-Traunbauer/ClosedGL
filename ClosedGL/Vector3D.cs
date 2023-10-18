using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL
{
    public struct Vector3D
    {
        public double X, Y, Z;

        public Vector3D(double x, double y, double z)
        {
            X = x; Y = y; Z = z;
        }

        public static Vector3D Forward = new Vector3D(0, 0, 1);
        public static Vector3D Up = new Vector3D(0, 1, 0);
        public static Vector3D Right = new Vector3D(1, 0, 0);

        public static Vector3D Backward = -Forward;
        public static Vector3D Down = -Up;
        public static Vector3D Left = -Right;

        public static Vector3D Zero { get; internal set; }

        public static Vector3D operator +(Vector3D a, Vector3D b)
        {
            return new Vector3D(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }

        public static Vector3D operator -(Vector3D a, Vector3D b)
        {
            return new Vector3D(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }

        public static Vector3D operator -(Vector3D a)
        {
            return new Vector3D(-a.X, -a.Y, -a.Z);
        }

        public static Vector3D operator *(Vector3D a, double b)
        {
            return new Vector3D(a.X * b, a.Y * b, a.Z * b);
        }

        public static Vector3D operator *(double b, Vector3D a)
        {
            return new Vector3D(a.X * b, a.Y * b, a.Z * b);
        }

        public static Vector3D operator /(Vector3D a, double b)
        {
            return new Vector3D(a.X / b, a.Y / b, a.Z / b);
        }

        public static Vector3D operator /(double b, Vector3D a)
        {
            return new Vector3D(a.X / b, a.Y / b, a.Z / b);
        }

        public static bool operator ==(Vector3D a, Vector3D b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
        }

        public static bool operator !=(Vector3D a, Vector3D b)
        {
            return !(a == b);
        }

        public static explicit operator Vector3D(Vector3 v)
        {
            return new Vector3D(v.X, v.Y, v.Z);
        }

        public static double Dot(Vector3D a, Vector3D b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static Vector3D Cross(Vector3D a, Vector3D b)
        {
            return new Vector3D(
                               a.Y * b.Z - a.Z * b.Y,
                                              a.Z * b.X - a.X * b.Z,
                                                             a.X * b.Y - a.Y * b.Z);
        }

        public double Length()
        {
            return Math.Sqrt(X * X + Y * Y + Z * Z);
        }

        public void Normalize()
        {
            this /= Length();
        }

        public static Vector3D Normalize(Vector3D normalize)
        {
            return normalize / normalize.Length();
        }

        public override string ToString()
        {
            return $"({X}, {Y}, {Z})";
        }
    }
}
