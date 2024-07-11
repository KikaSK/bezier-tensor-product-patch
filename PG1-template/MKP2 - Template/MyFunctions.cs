using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK;

namespace MKP2___Template
{
    static class MyFunctions
    {
        public static double Power(double x, int n)
        {
            if (n == 0) return 1;
            if (n == 1) return x;
            double pow = Power(x, n / 2);
            if (n % 2 == 0) return pow * pow;
            return x * pow * pow;
        }
        public static int CombNumber(int n, int k)
        {
            if (k == 0 || k == n) return 1;
            return CombNumber(n - 1, k - 1) + CombNumber(n - 1, k);
        }
        // Bernstein polynomial
        public static double B(double x, int k, int n)
        {
            return CombNumber(n, k) * Power(x, k) * Power(1 - x, n - k);
        }
        public static double[,] GetBezierMatrix(int size, bool V)
        { 
            double[,] result = new double[size, size];
            for(int i = 0; i<size; ++i)
            {
                for(int j = 0; j<size; ++j)
                {
                    if (V)
                        result[i, j] = B((double)i / (size - 1), j, size - 1);
                    else
                        result[i, j] = B((double)j / (size - 1), i, size - 1);
                }
            }
            return result;
        }
        
        // Multiply 2 matrices
        public static double[,] Multiply(ref double[,] M1, ref double[,] M2, int S1, int S12, int S2)
        {
            double[,] res = new double[S1, S2];
            for (int i = 0; i < S1; i++)
            {
                for (int j = 0; j < S2; j++)
                {
                    res[i, j] = 0;
                    for (int k = 0; k < S12; k++)
                    {
                        res[i, j] += M1[i, k] * M2[k, j];
                    }
                }
            }
            return res;
        }
        
        // DeCasteljau algorithm for curve
        public static Vector3 CurveDeCasteljau(float u, Vector3[]ControlPoints)
        {
            Vector3[] copyCV = ControlPoints.Clone() as Vector3[];
            for (int i = 0; i < ControlPoints.Count() - 1; ++i)
            {
                for (int j = 0; j < ControlPoints.Count() - i - 1; ++j)
                {
                    copyCV[j] = (1 - u) * copyCV[j] + u * copyCV[j + 1];
                }
            }
            return copyCV[0];
        }
        private static List<int> GetIndices(int eU, int eV)
        {
            List<int> IndList = new List<int>();

            // indices for rectangles - quadruples  
            if (eU <= 0)
            {
                for (int i = 0; i < eV; i++)
                {
                    IndList.Add(i);
                    IndList.Add(i);
                    IndList.Add(i + 1);
                    IndList.Add(i + 1);
                }
            }
            else if (eV <= 0)
            {
                for (int i = 0; i < eU; i++)
                {
                    IndList.Add(i);
                    IndList.Add(i + 1);
                    IndList.Add(i + 1);
                    IndList.Add(i);
                }
            }
            else
            {
                for (int i = 0; i < eV; i++)
                    for (int j = 0; j < eU; j++)
                    {
                        IndList.Add(i * (eU + 1) + j);
                        IndList.Add(i * (eU + 1) + j + 1);
                        IndList.Add((i + 1) * (eU + 1) + j + 1);
                        IndList.Add((i + 1) * (eU + 1) + j);
                    }
            }
            return IndList;
        }
        public static Vector3 BilinearDeCasteljau(int m, int n, float u, float v, List<Vector3> ControlPoints)
        {
            List<int> Indices = GetIndices(m - 1, n - 1);
            if (ControlPoints.Count == 1) return ControlPoints[0];
            List<Vector3> newControlPoints = new List<Vector3>();
            for (int i = 0; i < Indices.Count / 4; ++i)
            {
                int i1 = Indices[4 * i];
                int i2 = Indices[4 * i + 1];
                int i3 = Indices[4 * i + 2];
                int i4 = Indices[4 * i + 3];
                Vector3 newPoint = ControlPoints[i1] * (1 - v) * (1 - u) + ControlPoints[i4] * (1 - u) * v + ControlPoints[i2] * (1 - v) * u + ControlPoints[i3] * u * v;
                newControlPoints.Add(newPoint);
            }
            return BilinearDeCasteljau(m - 1, n - 1, u, v, newControlPoints);
        }
        public static Vector3 GetUnitNormal(int m, int n, float u, float v, List<Vector3> ControlPoints)
        {
            List<Vector3> DifferenceU = new List<Vector3>();
            for (int i = 0; i < n - 1; ++i)
                for (int j = 0; j < m; ++j)
                    DifferenceU.Add(ControlPoints[(i + 1) * m + j] - ControlPoints[i * m + j]);
           
            List<Vector3> DifferenceV = new List<Vector3>();
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < m - 1; ++j)
                    DifferenceV.Add(ControlPoints[i * m + j + 1] - ControlPoints[i * m + j]);

            Vector3 dU = BilinearDeCasteljau(m, n-1, u, v, DifferenceU);
            Vector3 dV = BilinearDeCasteljau(m - 1, n, u, v, DifferenceV);

            return Vector3.Cross(dU, dV).Normalized();
        }

        public static void GetSamplingAndNormals(ref List<Vector3> Sampling, ref List<Vector3> Normals, ref List<Vector3> ControlPoints, int m, int n)
        {
            for (int i = 0; i < Sampling.Count; ++i)
            {
                var sample = Sampling[i];
                float u = (sample.X + 1) / 2;
                float v = (sample.Y + 1) / 2;
                Vector3 res = BilinearDeCasteljau(m, n, u, v, ControlPoints);
                Sampling[i] = res;
                Vector3 normal = GetUnitNormal(m, n, u, v, ControlPoints);
                Normals.Add(normal);
            }
        }

        public static void GetIsoControl(ref List<Vector3> iso, float u, int n, List<Vector3[]> coord)
        {
            iso.Clear();
            for (int j = 0; j < n; ++j)
            {
                iso.Add(MyFunctions.CurveDeCasteljau(u, coord[j]));
            }
        }
        public static void GetIsoSampling(ref List<Vector3>isoS, ref List<Vector3> isoT, ref List<Vector3>control, int NoS, int n, int m, float s, float t)
        {
            isoS.Clear();
            isoT.Clear();

            for (int i = 0; i <= NoS; ++i)
            {
                float u = (float)i / NoS;
                Vector3 res = BilinearDeCasteljau(m, n, u, s, control);
                isoT.Add(res);
                res = BilinearDeCasteljau(m, n, t, u, control);
                isoS.Add(res);
            }

        }
    }
}
