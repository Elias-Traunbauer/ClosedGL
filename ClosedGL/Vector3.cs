using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL
{
    public struct Vector3
    {
        public float X, Y, Z;

        public Vector3(float x, float y, float z)
        {
            X = x; Y = y; Z = z;
        }

        public static Vector3 Forward = new Vector3(0, 0, 1);
        public static Vector3 Up = new Vector3(0, 1, 0);
        public static Vector3 Right = new Vector3(1, 0, 0);

        public static Vector3 Backward = -Forward;
        public static Vector3 Down = -Up;
        public static Vector3 Left = -Right;

        public static Vector3 operator +(Vector3 a, Vector3 b)
        {
            return new Vector3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }

        public static Vector3 operator -(Vector3 a, Vector3 b)
        {
            return new Vector3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }

        public static Vector3 operator -(Vector3 a)
        {
            return new Vector3(-a.X, -a.Y, -a.Z);
        }

        public static Vector3 operator *(Vector3 a, double b)
        {
            return new Vector3(a.X * b, a.Y * b, a.Z * b);
        }

        public static Vector3 operator *(double b, Vector3 a)
        {
            return new Vector3(a.X * b, a.Y * b, a.Z * b);
        }

        public static Vector3 operator /(Vector3 a, double b)
        {
            return new Vector3(a.X / b, a.Y / b, a.Z / b);
        }

        public static Vector3 operator /(double b, Vector3 a)
        {
            return new Vector3(a.X / b, a.Y / b, a.Z / b);
        }

        public static bool operator ==(Vector3 a, Vector3 b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
        }

        public static bool operator !=(Vector3 a, Vector3 b)
        {
            return !(a == b);
        }

        public static double Dot(Vector3 a, Vector3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static Vector3 Cross(Vector3 a, Vector3 b)
        {
            return new Vector3(
                               a.Y * b.Z - a.Z * b.Y,
                                              a.Z * b.X - a.X * b.Z,
                                                             a.X * b.Y - a.Y * b.Z);
        }

        public double Length()
        {
            return Math.Sqrt(X * X + Y * Y + Z * Z);
        }

        public Vector3 Normalize()
        {
            return this / Length();
        }

        public override string ToString()
        {
            return $"({X}, {Y}, {Z})";
        }

        public static void Transform(ref Vector3 up, ref System.Numerics.Quaternion rot, out Vector3 rotatedUp)
        {
            var q = new System.Numerics.Quaternion(up.X, up.Y, up.Z, 0);
            var r = rot * q * System.Numerics.Quaternion.Conjugate(rot);
            rotatedUp = new Vector3(r.X, r.Y, r.Z);
        }

        internal static void TransformNormal(ref Vector3 forward, ref Matrix rotation, out Vector3 rotatedForward)
        {
            throw new NotImplementedException();
        }
    }
}
