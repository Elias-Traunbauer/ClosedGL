using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL.SMath
{
    public class PerlinNoise
    {
        private const int GradientSizeTable = 256;
        private readonly Random _random;
        private readonly byte[] _permutations;

        // Gradients for 2D. They approximate the directions to the vertices of an octagon from the center.
        private static readonly float[][] Gradients = {
        new[] { 1f, 0f },
        new[] { -1f, 0f },
        new[] { 0f, 1f },
        new[] { 0f, -1f },
        new[] { 0.707f, 0.707f },
        new[] { -0.707f, 0.707f },
        new[] { 0.707f, -0.707f },
        new[] { -0.707f, -0.707f }
    };

        public PerlinNoise(int seed)
        {
            _random = new Random(seed);
            _permutations = new byte[GradientSizeTable * 2];

            // Fill the permutations table
            for (int i = 0; i < GradientSizeTable; i++)
                _permutations[i] = (byte)i;

            // Shuffle the array
            for (int i = 0; i < GradientSizeTable; i++)
            {
                int j = _random.Next(GradientSizeTable);

                // Swap
                (_permutations[i], _permutations[j]) = (_permutations[j], _permutations[i]);
            }

            // Duplicate the permutations in the second half of the table.
            for (int i = 0; i < GradientSizeTable; i++)
                _permutations[GradientSizeTable + i] = _permutations[i];
        }

        public float Generate(float x, float y)
        {
            // Compute the cell coordinates
            int x0 = x > 0.0 ? (int)x : (int)x - 1;
            int y0 = y > 0.0 ? (int)y : (int)y - 1;

            // Relative x, y in cell
            x -= x0;
            y -= y0;

            // Wrap the integer cells at 255 (smaller integer period can be introduced here)
            x0 &= 255;
            y0 &= 255;

            // Calculate noise contributions from each of the four corners
            float n00 = Dot(Gradients[_permutations[x0 + _permutations[y0]] % 8], x, y);
            float n01 = Dot(Gradients[_permutations[x0 + _permutations[y0 + 1]] % 8], x, y - 1);
            float n10 = Dot(Gradients[_permutations[x0 + 1 + _permutations[y0]] % 8], x - 1, y);
            float n11 = Dot(Gradients[_permutations[x0 + 1 + _permutations[y0 + 1]] % 8], x - 1, y - 1);

            // Smooth the noise with a fade function
            float u = Fade(x);
            float v = Fade(y);

            // Interpolate the four results
            return Lerp(
                Lerp(n00, n10, u),
                Lerp(n01, n11, u),
                v);
        }

        public float Generate01(float x, float y)
        {
            // Compute the cell coordinates
            int x0 = x > 0.0 ? (int)x : (int)x - 1;
            int y0 = y > 0.0 ? (int)y : (int)y - 1;

            // Relative x, y in cell
            x -= x0;
            y -= y0;

            // Wrap the integer cells at 255 (smaller integer period can be introduced here)
            x0 &= 255;
            y0 &= 255;

            // Calculate noise contributions from each of the four corners
            float n00 = Dot(Gradients[_permutations[x0 + _permutations[y0]] % 8], x, y);
            float n01 = Dot(Gradients[_permutations[x0 + _permutations[y0 + 1]] % 8], x, y - 1);
            float n10 = Dot(Gradients[_permutations[x0 + 1 + _permutations[y0]] % 8], x - 1, y);
            float n11 = Dot(Gradients[_permutations[x0 + 1 + _permutations[y0 + 1]] % 8], x - 1, y - 1);

            // Smooth the noise with a fade function
            float u = Fade(x);
            float v = Fade(y);

            // Interpolate the four results
            return Lerp(
                               Lerp(n00, n10, u),
                                              Lerp(n01, n11, u),
                                                             v) * 0.5f + 0.5f;
        }

        private float Dot(float[] g, float x, float y)
        {
            return g[0] * x + g[1] * y;
        }

        private float Lerp(float a, float b, float t)
        {
            return a + t * (b - a);
        }

        private float Fade(float t)
        {
            // Smoothstep function
            return t * t * t * (t * (t * 6 - 15) + 10);
        }
    }
}
