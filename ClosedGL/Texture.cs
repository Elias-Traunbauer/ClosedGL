using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL
{
    public class Texture
    {
        public byte[] Data { get; set; }
        public int Width { get; set; }
        public int Height { get; set; }
        public Texture? Mirror { get; set; } = null;
        public Texture(int width, int height)
        {
            Width = width;
            Height = height;
            Data = new byte[width * height * 4];
        }
        public static Dictionary<string, Texture> loadedTextures = new();
        public Texture(string path) 
        {
            if (loadedTextures.ContainsKey(path))
            {
                Mirror = loadedTextures[path];
                Data = new byte[0];
                Width = Mirror.Width;
                Height = Mirror.Height;
                return;
            }
            Bitmap bitmap = new(path);
            Width = bitmap.Width;
            Height = bitmap.Height;
            Data = new byte[Width * Height * 4];
            var lockData = bitmap.LockBits(new Rectangle(0, 0, Width, Height), System.Drawing.Imaging.ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
            System.Runtime.InteropServices.Marshal.Copy(lockData.Scan0, Data, 0, Width * Height * 4);
            bitmap.UnlockBits(lockData);
            bitmap.Dispose();
            loadedTextures[path] = this;
        }

        public Color GetPixel(int textureX, int textureY)
        {
            if (Mirror != null)
            {
                return Mirror.GetPixel(textureX, textureY);
            }

            int index = (textureY * Width + textureX) * 4;
            // bitmap data is stored as BGRA
            
            byte b = Data[index];
            byte g = Data[index + 1];
            byte r = Data[index + 2];
            byte a = Data[index + 3];

            return Color.FromArgb(a, r, g, b);
        }

        public byte[] GetPixelAsBytes(int textureX, int textureY)
        {
            if (Mirror != null)
            {
                return Mirror.GetPixelAsBytes(textureX, textureY);
            }

            int index = (textureY * Width + textureX) * 4;
            // bitmap data is stored as BGRA

            byte b = Data[index];
            byte g = Data[index + 1];
            byte r = Data[index + 2];
            byte a = Data[index + 3];

            return new byte[] { b, g, r, a };
        }

        public int DataLength()
        {
            return Data.Length;
        }
    }
}
