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
        public Texture(int width, int height)
        {
            Width = width;
            Height = height;
            Data = new byte[width * height * 4];
        }

        public Color GetPixel(int textureX, int textureY)
        {
            int index = (textureY * Width + textureX) * 4;
            byte r = Data[index];
            byte g = Data[index + 1];
            byte b = Data[index + 2];
            byte a = Data[index + 3];
            return Color.FromArgb(a, r, g, b);
        }

        public byte[] GetPixelAsBytes(int textureX, int textureY)
        {
            int index = (textureY * Width + textureX) * 4;
            byte r = Data[index];
            byte g = Data[index + 1];
            byte b = Data[index + 2];
            byte a = Data[index + 3];
            return new byte[] { r, g, b, a };
        }


    }
}
