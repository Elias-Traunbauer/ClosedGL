using VRageMath;

namespace RenderTest
{
    internal struct YawPitchRoll
    {
        public float Yaw;
        public float Pitch;
        public float Roll;

        public YawPitchRoll(float yaw, float pitch, float roll)
        {
            Yaw = yaw;
            Pitch = pitch;
            Roll = roll;
        }

        public Quaternion ToQuaternion()
        {
            float cy = (float)System.Math.Cos(Yaw * 0.5f);
            float sy = (float)System.Math.Sin(Yaw * 0.5f);
            float cp = (float)System.Math.Cos(Pitch * 0.5f);
            float sp = (float)System.Math.Sin(Pitch * 0.5f);
            float cr = (float)System.Math.Cos(Roll * 0.5f);
            float sr = (float)System.Math.Sin(Roll * 0.5f);

            Quaternion q;
            q.W = cy * cp * cr + sy * sp * sr;
            q.X = cy * cp * sr - sy * sp * cr;
            q.Y = sy * cp * sr + cy * sp * cr;
            q.Z = sy * cp * cr - cy * sp * sr;
            return q;
        }

        internal static YawPitchRoll FromQuaternion(Quaternion rotation)
        {
            float yaw = 0;
            float pitch = 0;
            float roll = 0;

            float test = rotation.X * rotation.Y + rotation.Z * rotation.W;
            if (test > 0.499f)
            {
                yaw = 2 * (float)System.Math.Atan2(rotation.X, rotation.W);
                pitch = (float)System.Math.PI / 2;
                roll = 0;
            }
            else if (test < -0.499f)
            {
                yaw = -2 * (float)System.Math.Atan2(rotation.X, rotation.W);
                pitch = (float)-System.Math.PI / 2;
                roll = 0;
            }
            else
            {
                float sqx = rotation.X * rotation.X;
                float sqy = rotation.Y * rotation.Y;
                float sqz = rotation.Z * rotation.Z;

                yaw = (float)System.Math.Atan2(2 * rotation.Y * rotation.W - 2 * rotation.X * rotation.Z, 1 - 2 * sqy - 2 * sqz);
                pitch = (float)System.Math.Asin(2 * test);
                roll = (float)System.Math.Atan2(2 * rotation.X * rotation.W - 2 * rotation.Y * rotation.Z, 1 - 2 * sqx - 2 * sqz);
            }

            return new YawPitchRoll(yaw, pitch, roll);
        }
    }
}