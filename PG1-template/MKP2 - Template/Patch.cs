using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK;

namespace MKP2___Template
{
    // enum type  determines the type of a patch
    public enum type
    {
        TENSOR,  // interpolating Bezier tensor product patch
        SPHERE, // one eighth of a sphere
        CYLINDER, // one fourth of a cylinder
        CONE // one fourth of a cone
    };

    // enum placement determines the position of a patch
    public enum placement 
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

    class Patch
    {
        // Class CNet describes the control vertices of a patch
        public class CNet
        {
            public List<int> Indices;
            public List<Vector3> Coordinates;

            public CNet()
            {
                Indices = new List<int>(); // Indices of a control net
                Coordinates = new List<Vector3>(); // Coordinates of vertices of the control net
            }
        }

        // Class IPoints describes the interpolated points of a patch
        public class IPoints
        {
            public List<Vector3> Coordinates;

            public IPoints()
            {
                Coordinates = new List<Vector3>(); // Coordinates of interpolated points
            }
        }

        // sampled patch (points computed using the de Casteljau algorithm)
        public class Sampl
        {
            public List<int> Indices; 
            public List<Vector3> Coordinates;
            public List<Vector3> Normals;

            public Sampl()
            {
                Indices = new List<int>();
                Coordinates = new List<Vector3>();
                Normals = new List<Vector3>();
            }
        }

        public type TypeOfPatch; 
        public placement Place;
        public int NumberOfSamples, DegreeM, DegreeN;

        public CNet ControlNet;
        public IPoints InterpolatedPoints;
        public Sampl Sampling;
        public float[] Color;

        public bool? DisplayIsoCurves;

        public double hlS, hlT; // parameter S and T, respectively, for the isocurves

        public List<Vector3> IsoS_ControlPolygon, IsoT_ControlPolygon; // coordinates of the vertices of the control polygon of the isocurves
        public List<Vector3> IsoS_Sampling, IsoT_Sampling;  // coordinates of the sampled points of the isocurves

        // Initialization of a patch
        public Patch(type _TypeOfPatch, int _DegreeM, int _DegreeN, int _NumberOfSamples, bool? _DisplayIsoCurves, double _hlS, double _hlT, float[] _Color, placement _Place)
        {
            TypeOfPatch = _TypeOfPatch;
            NumberOfSamples = _NumberOfSamples;
            DegreeM = _DegreeM;
            DegreeN = _DegreeN;
            Color = _Color;
            Place = _Place;

            DisplayIsoCurves = _DisplayIsoCurves;
            hlS = _hlS;
            hlT = _hlT;

            ControlNet = new CNet();
            Sampling = new Sampl();
            InterpolatedPoints = new IPoints();

            IsoS_ControlPolygon = new List<Vector3>();
            IsoT_ControlPolygon = new List<Vector3>();
            IsoS_Sampling = new List<Vector3>();
            IsoT_Sampling = new List<Vector3>();

            // Initial sampling of a patch
            Sampling.Coordinates = Sample(TypeOfPatch, NumberOfSamples, NumberOfSamples);
            Sampling.Normals = GetNormals(TypeOfPatch, NumberOfSamples, NumberOfSamples);
            Sampling.Indices = GetIndices(TypeOfPatch, NumberOfSamples, NumberOfSamples, true);

            // Initial control net
            ControlNet.Coordinates = Sample(TypeOfPatch, DegreeM, DegreeN);
            ControlNet.Indices = GetIndices(TypeOfPatch, DegreeM, DegreeN, false);

            // Initial interpolated vertices
            InterpolatedPoints.Coordinates = Sample(TypeOfPatch, DegreeM, DegreeN);
            
        }

        // sampling of the initial patch with the given number of samples eU, eV in the direction u, v, respectively
        private List<Vector3> Sample(type _TypeOfPatch, int eU, int eV)
        {
            List<Vector3> SampleList = new List<Vector3>();
            
                for (int i = 0; i <= eV; i++)
                    for (int j = 0; j <= eU; j++)
                        SampleList.Add(new Vector3(-1.0f + 2.0f * i / eV, -1.0f + 2.0f * j / eU, 0.0f));                    

            return SampleList;
        }

        // get default normals for each vertex for the flat patch
        private List<Vector3> GetNormals(type _TypeOfPatch, int eU, int eV)
        {
            List<Vector3> NormalList = new List<Vector3>();

            for (int i = 0; i <= eV; i++)
                for (int j = 0; j <= eU; j++)
                    NormalList.Add(new Vector3(0.0f, 0.0f, 1.0f));


            return NormalList;
        }

        // getting indicies for a patch with given number of samples eU, eV in the direction u, v, respectively 
        private List<int> GetIndices(type _TypeOfPatch, int eU, int eV, bool DrawAll)
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

        // Indices of the sampled grid are in the following order (example is the patch with degM = 3, degN = 2
        //
        //      8 --- 9 -- 10 -- 11
        //      |     |     |     |
        //      4 --- 5 --- 6 --- 7
        //      |     |     |     |
        //      0 --- 1 --- 2 --- 3
        //
        //

        // Computation of points of the patch 
        public void RecomputePatch()
        {
            if (TypeOfPatch == type.TENSOR)
            {
                // --------------- !!! TODO !!! -------------------
                //
                // Get the coordinates of the control net of the INTERPOLATING BEZIER PATCH, which are stored in the vector "ControlNet.Coordinates"
                //
                // Do not forget to clear the vector ControlNet.Coordinates beforehand
                //
                // ------------------------------------------------

                int m = DegreeM + 1;
                int n = DegreeN + 1;

                // heights of interpolated points
                double[,] heights = new double[n, m];
                for (int i = 0; i < n; ++i)
                {
                    for (int j = 0; j < m; ++j)
                    {
                        heights[i, j] = InterpolatedPoints.Coordinates[i * m + j].Z;
                    }
                }

                // matrices of bernstein polynomials evaluated in Greville abscissae
                double[,] uMatrix = MyFunctions.GetBezierMatrix(m, false);
                double[,] vMatrix = MyFunctions.GetBezierMatrix(n, true);

                // inverse matrices
                alglib.rmatrixinverse(ref uMatrix, out int uInfo, out alglib.matinvreport uReport);
                alglib.rmatrixinverse(ref vMatrix, out int vInfo, out alglib.matinvreport vReport);

                if (vInfo == -3 || uInfo == -3)
                {
                    throw new Exception("ERROR: Computing inverse of singular matrix!");
                }
                else
                {
                    // mutiplication of right side by inverse to obtain heights of control points
                    double[,] temp = MyFunctions.Multiply(ref vMatrix, ref heights, n, n, m);
                    double[,] result = MyFunctions.Multiply(ref temp, ref uMatrix, n, m, m);

                    ControlNet.Coordinates.Clear();
                    for (int i = 0; i < n; ++i)
                    {
                        for (int j = 0; j < m; ++j)
                        {
                            ControlNet.Coordinates.Add(new Vector3(-1.0f + 2.0f * i / DegreeN, -1.0f + 2.0f * j / DegreeM, (float)result[i, j]));
                        }
                    }
                }
            }
            if (TypeOfPatch == type.SPHERE)
            {
                // --------------- !!! TODO !!! -------------------
                //
                // Get the coordinates of the control net of the SPHERE
                //
                // Do not forget to clear the vector ControlNet.Coordinates beforehand
                //
                // ------------------------------------------------

                DegreeM = 2;
                DegreeN = 2;

                Vector3 p00 = new Vector3(1, -1, -1);
                Vector3 p10 = new Vector3(1, -1, -1);
                Vector3 p20 = new Vector3(1, -1, -1);

                Vector3 p02 = new Vector3(-1, -1, 1);
                Vector3 p22 = new Vector3(-1, 1, -1);

                Vector3 p01 = new Vector3(1, -1, 1);
                Vector3 p21 = new Vector3(1, 1, -1);
                Vector3 p12 = new Vector3(-1, 1, 1);

                Vector3 p11 = new Vector3(1, 1, 1);

                ControlNet.Coordinates.Clear();
                ControlNet.Coordinates.Add(p00);
                ControlNet.Coordinates.Add(p01);
                ControlNet.Coordinates.Add(p02);

                ControlNet.Coordinates.Add(p10);
                ControlNet.Coordinates.Add(p11);
                ControlNet.Coordinates.Add(p12);

                ControlNet.Coordinates.Add(p20);
                ControlNet.Coordinates.Add(p21);
                ControlNet.Coordinates.Add(p22);

                #region DEGREE3SPHERE
                /*
                DegreeM = 3;
                DegreeN = 3;
                Vector3 p00 = new Vector3(2, 0, 0);
                Vector3 p10 = new Vector3(2, 0, 0);
                Vector3 p20 = new Vector3(2, 0, 0);
                Vector3 p30 = new Vector3(2, 0, 0);

                Vector3 p03 = new Vector3(0, 0, 2);
                Vector3 p33 = new Vector3(0, 2, 0);

                float k = (float)(Math.Tan(Math.PI / 8)*2*4/3);

                Vector3 p01 = new Vector3(2, 0, k);
                Vector3 p02 = new Vector3(k, 0, 2);
                
                Vector3 p13 = new Vector3(0, k, 2);
                Vector3 p23 = new Vector3(0, 2, k);

                Vector3 p31 = new Vector3(2, k, 0);
                Vector3 p32 = new Vector3(k, 2, 0);

                float cos30 = (float)Math.Cos(Math.PI / 6);
                float sin30 = (float)Math.Sin(Math.PI / 6);

                Vector3 p11 = new Vector3(2, k * sin30, k * cos30);
                Vector3 p12 = new Vector3(k, 2 * sin30, 2 * cos30);

                Vector3 p21 = new Vector3(2, k * cos30, k * sin30);
                Vector3 p22 = new Vector3(k, 2 * cos30, 2 * sin30);

                Vector3 shift = new Vector3(-1, -1, -1);

                ControlNet.Coordinates.Clear();
                ControlNet.Coordinates.Add(p00 + shift);
                ControlNet.Coordinates.Add(p01 + shift);
                ControlNet.Coordinates.Add(p02 + shift);
                ControlNet.Coordinates.Add(p03 + shift);

                ControlNet.Coordinates.Add(p10 + shift);
                ControlNet.Coordinates.Add(p11 + shift);
                ControlNet.Coordinates.Add(p12 + shift);
                ControlNet.Coordinates.Add(p13 + shift);

                ControlNet.Coordinates.Add(p20 + shift);
                ControlNet.Coordinates.Add(p21 + shift);
                ControlNet.Coordinates.Add(p22 + shift);
                ControlNet.Coordinates.Add(p23 + shift);

                ControlNet.Coordinates.Add(p30 + shift);
                ControlNet.Coordinates.Add(p31 + shift);
                ControlNet.Coordinates.Add(p32 + shift);
                ControlNet.Coordinates.Add(p33 + shift);
                */
                #endregion

                ControlNet.Indices = GetIndices(TypeOfPatch, DegreeM, DegreeN, false);
            }

            if (TypeOfPatch == type.CYLINDER)
            {
                // --------------- !!! TODO !!! -------------------
                //
                // Get the coordinates of the control net of the CYLINDER
                //
                // Do not forget to clear the vector "ControlNet.Coordinates" beforehand
                //
                // ------------------------------------------------

                DegreeM = 1;
                DegreeN = 2;

                Vector3 p00 = new Vector3(-1, -1, 1);
                Vector3 p01 = new Vector3(-1, 1, 1);
                Vector3 p02 = new Vector3(1, 1, 1);

                Vector3 p10 = new Vector3(-1, -1, -1);
                Vector3 p11 = new Vector3(-1, 1, -1);
                Vector3 p12 = new Vector3(1, 1, -1);

                ControlNet.Coordinates.Clear();
                ControlNet.Coordinates.Add(p00);
                ControlNet.Coordinates.Add(p10);

                ControlNet.Coordinates.Add(p01);
                ControlNet.Coordinates.Add(p11);

                ControlNet.Coordinates.Add(p02);
                ControlNet.Coordinates.Add(p12);

                ControlNet.Indices = GetIndices(TypeOfPatch, DegreeM, DegreeN, false);
            }

            if (TypeOfPatch == type.CONE)
            {
                // --------------- !!! TODO !!! -------------------
                //
                // Get the coordinates of the control net of the CONE
                //
                // Do not forget to clear the vector "ControlNet.Coordinates" beforehand
                //
                // ------------------------------------------------

                DegreeM = 1;
                DegreeN = 2;

                Vector3 p00 = new Vector3(-1, -1, 1);
                Vector3 p01 = new Vector3(-1, -1, 1);
                Vector3 p02 = new Vector3(-1, -1, 1);

                Vector3 p10 = new Vector3(-1, 1, -1);
                Vector3 p11 = new Vector3(1, 1, -1);
                Vector3 p12 = new Vector3(1, -1, -1);

                ControlNet.Coordinates.Clear();
                ControlNet.Coordinates.Add(p00);
                ControlNet.Coordinates.Add(p10);

                ControlNet.Coordinates.Add(p01);
                ControlNet.Coordinates.Add(p11);

                ControlNet.Coordinates.Add(p02);
                ControlNet.Coordinates.Add(p12);

                ControlNet.Indices = GetIndices(TypeOfPatch, DegreeM, DegreeN, false);
            }

            // --------------- !!! TODO !!! -------------------
            //
            // Using the control net compute the samples of the patch using the DE CASTELJAU ALGORITHM
            // Also, compute the **unit** normal vector for each of the samples
            //
            // Do not forget to clear the vectors  "Sampling.Coordinates" and "Sampling.Normals" beforehand
            //
            // ------------------------------------------------       

            Sampling.Coordinates.Clear();
            Sampling.Coordinates = Sample(TypeOfPatch, NumberOfSamples, NumberOfSamples);
            Sampling.Normals.Clear();

            MyFunctions.GetSamplingAndNormals(ref Sampling.Coordinates, ref Sampling.Normals, ref ControlNet.Coordinates, DegreeM+1, DegreeN+1);
            
            if(DisplayIsoCurves == true)
            {
                // --------------- !!! TODO !!! -------------------
                //
                // Compute the control polygon and sampled points of the S-isocurve and T-isocurve
                //
                // Do not forget to clear the vectors "IsoT_ControlPolygon", "IsoT_Sampling", "IsoS_ControlPolygon" and "IsoS_Sampling" beforehand
                //
                // ------------------------------------------------
                int n = DegreeN + 1;
                int m = DegreeM + 1;

                float s = (float)hlS;
                float t = (float)hlT;

                MyFunctions.GetIsoSampling(ref IsoS_Sampling, ref IsoT_Sampling, ref ControlNet.Coordinates, NumberOfSamples, n, m, s, t);

                List<Vector3[]> ControlPoints = new List<Vector3[]>();
                for (int i = 0; i < n; ++i)
                {
                    ControlPoints.Add(new Vector3[m]);
                    for (int j = 0; j < m; ++j)
                        ControlPoints[i][j] = ControlNet.Coordinates[i * m + j];
                }

                MyFunctions.GetIsoControl(ref IsoS_ControlPolygon, t, n, ControlPoints);

                ControlPoints.Clear();
                for (int j = 0; j < m; ++j)
                {
                    ControlPoints.Add(new Vector3[n]);
                    for (int i = 0; i < n; ++i)
                        ControlPoints[j][i] = ControlNet.Coordinates[i * m + j];
                }

                MyFunctions.GetIsoControl(ref IsoT_ControlPolygon, s, m, ControlPoints);     
            }
        }        
        
        
    }
}
