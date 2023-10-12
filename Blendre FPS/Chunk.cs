// Chunk is the basic unit for terrain modification.
// many functions support parallel processing
// chunk consists of many small grids

using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;


public class Chunk
{
    public static int chunkSize = 10;
    public static float gridLength = 0.5f;
    public static Gradient gradient;
    public static int mode = 0;

    int xPos, yPos, zPos;
    public Vector3 chunkBase;
    public Vector3Int chunkNum;

    public MeshCollider mc;
    public MeshFilter mf;
    public MeshRenderer mr;
    List<Vector3> vertices;
    List<int> triangles;
    List<Color> colors;
    private object genMesh = new object();
    private object calVert = new object();

    public Chunk(GameObject chunkObj, int x, int y, int z)
    {
        mc = chunkObj.GetComponent<MeshCollider>();
        mf = chunkObj.GetComponent<MeshFilter>();
        mr = chunkObj.GetComponent<MeshRenderer>();

        vertices = new List<Vector3>();
        triangles = new List<int>();
        colors = new List<Color>();
        xPos = x;
        yPos = y;
        zPos = z;
        chunkNum = new Vector3Int(x, y, z);
        chunkBase = new Vector3(xPos * chunkSize, yPos * chunkSize, zPos * chunkSize);
    }

    // Chunk initialization using the seed and simplex noise
    // overlay noises to achieve complex terrain
    public void InitMap(object obj)
    {
        float frequency, amplitude;
        int gw = MapManager.gridOnWidth;
        int gh = MapManager.gridOnHeight;
        int gl = MapManager.gridOnLength;
        int init_x = (int)chunkBase.x;
        int init_y = (int)chunkBase.y;
        int init_z = (int)chunkBase.z;
        Vector3 p;
        for (int w = init_x; w < init_x + chunkSize; w++)
        {
            for (int h = init_y; h < init_y + chunkSize; h++)
            {
                for (int l = init_z; l < init_z + chunkSize; l++)
                {
                    if (l == 0 || h == 0 || w == 0)
                    {
                        MapManager.gridVal[w, h, l] = 1;
                        continue;
                    }
                    if (h < 2)
                    {
                        MapManager.gridVal[w, h, l] = -1;
                        continue;
                    }
                    if (mode == 0) // flat mode
                    {
                        frequency = 1;
                        amplitude = 20;
                        float noise = 0;
                        p = MapManager.seed + new Vector3((float)w / gw, (float)l / gl, (float)h / gh);
                        for (int i = 0; i < 7; i++)
                        {
                            noise += Noise.Simplex2D(p, frequency).value * amplitude;
                            frequency *= 2;
                            amplitude *= 0.5f;
                        }
                        MapManager.gridVal[w, h, l] = (h - 30) + noise;
                    }
                    else if (mode == 1) // 3d mode
                    {
                        frequency = 2;
                        amplitude = 1;
                        float noise = 0;
                        p = MapManager.seed + new Vector3((float)w / gw, (float)h / gh, (float)l / gl);
                        for (int i = 0; i < 4; i++)
                        {
                            noise += Noise.Simplex3D(p, frequency).value * amplitude;
                            frequency *= 2;
                            amplitude *= 0.5f;
                        }
                        MapManager.gridVal[w, h, l] = -0.2f + noise;
                    }
                }
            }
        }
        // inform map manager a chunk is ready
        Interlocked.Increment(ref MapManager.readyCount);
    }


    // an atom function
    // calculate vertex positions from the implicit expression
    public void CalculateVerts(object obj)
    {
        lock (calVert)
        {
            vertices.Clear();
            triangles.Clear();
            colors.Clear();
            Vector3 current = Vector3.zero;
            int[] triVerts = new int[3];
            Vector3[] vertlist = new Vector3[12];
            float[] vertVal = new float[8];
            Dictionary<int, int> vertDict = new Dictionary<int, int>();
            int sum, offset, triIndex, sharedIndex;
            float val;
            Vector3 vert;
            offset = 0;
            triIndex = 0;
            for (int x = 0; x < chunkSize; x++)
            {
                current.x = x;
                for (int z = 0; z < chunkSize; z++)
                {
                    current.z = z;
                    for (int y = 0; y < chunkSize; y++)
                    {
                        current.y = y;
                        sum = 0;
                        for (int i = 0; i < 8; i++)
                        {
                            vert = MarchingCubes.vertices[i] + chunkBase;
                            try
                            {
                                val = MapManager.gridVal[x + (int)vert.x, y + (int)vert.y, z + (int)vert.z];
                                vertVal[i] = val;
                                if (val < 0) sum |= 1 << i;
                            }
                            catch
                            {
                                vertVal[i] = 1;
                            }

                        }

                        // Marching cube look up
                        if ((MarchingCubes.edgeTable[sum] & 1) > 0)
                            vertlist[0] =
                               VertexInterp(0, 1, vertVal[0], vertVal[1]);
                        if ((MarchingCubes.edgeTable[sum] & 2) > 0)
                            vertlist[1] =
                               VertexInterp(1, 2, vertVal[1], vertVal[2]);
                        if ((MarchingCubes.edgeTable[sum] & 4) > 0)
                            vertlist[2] =
                               VertexInterp(2, 3, vertVal[2], vertVal[3]);
                        if ((MarchingCubes.edgeTable[sum] & 8) > 0)
                            vertlist[3] =
                               VertexInterp(3, 0, vertVal[3], vertVal[0]);
                        if ((MarchingCubes.edgeTable[sum] & 16) > 0)
                            vertlist[4] =
                               VertexInterp(4, 5, vertVal[4], vertVal[5]);
                        if ((MarchingCubes.edgeTable[sum] & 32) > 0)
                            vertlist[5] =
                               VertexInterp(5, 6, vertVal[5], vertVal[6]);
                        if ((MarchingCubes.edgeTable[sum] & 64) > 0)
                            vertlist[6] =
                               VertexInterp(6, 7, vertVal[6], vertVal[7]);
                        if ((MarchingCubes.edgeTable[sum] & 128) > 0)
                            vertlist[7] =
                               VertexInterp(7, 4, vertVal[7], vertVal[4]);
                        if ((MarchingCubes.edgeTable[sum] & 256) > 0)
                            vertlist[8] =
                               VertexInterp(0, 4, vertVal[0], vertVal[4]);
                        if ((MarchingCubes.edgeTable[sum] & 512) > 0)
                            vertlist[9] =
                               VertexInterp(1, 5, vertVal[1], vertVal[5]);
                        if ((MarchingCubes.edgeTable[sum] & 1024) > 0)
                            vertlist[10] =
                               VertexInterp(2, 6, vertVal[2], vertVal[6]);
                        if ((MarchingCubes.edgeTable[sum] & 2048) > 0)
                            vertlist[11] =
                               VertexInterp(3, 7, vertVal[3], vertVal[7]);

                        for (int i = 0; i < 16; i += 3)
                        {
                            if (MarchingCubes.triTable[sum, i] == -1) break;
                            triVerts[0] = MarchingCubes.triTable[sum, i + 2];
                            triVerts[1] = MarchingCubes.triTable[sum, i + 1];
                            triVerts[2] = MarchingCubes.triTable[sum, i];

                            vertices.Add((vertlist[triVerts[0]] + current) * gridLength);
                            colors.Add(gradient.Evaluate((vertlist[triVerts[0]] + current + chunkBase).y / 60));
                            vertices.Add((vertlist[triVerts[1]] + current) * gridLength);
                            colors.Add(gradient.Evaluate((vertlist[triVerts[1]] + current + chunkBase).y / 60));
                            vertices.Add((vertlist[triVerts[2]] + current) * gridLength);
                            colors.Add(gradient.Evaluate((vertlist[triVerts[2]] + current + chunkBase).y / 60));
                            triangles.Add(vertices.Count - 3);
                            triangles.Add(vertices.Count - 2);
                            triangles.Add(vertices.Count - 1);
                        }
                        offset += 1;
                    }
                }
            }
            // add to refreshbuffer for later batch process
            MapManager.refreshBuffer.Enqueue(this);
        }
    }

    Vector3 VertexInterp(int p1, int p2, float v1, float v2)
    {
        Vector3 P1 = MarchingCubes.vertices[p1];
        Vector3 P2 = MarchingCubes.vertices[p2];
        if (Mathf.Abs(0 - v1) < 0.00001f)
            return P1;
        if (Mathf.Abs(0 - v2) < 0.00001f)
            return P2;
        if (Mathf.Abs(v1 - v2) < 0.00001f)
            return P1;
        float mu = (0 - v1) / (v2 - v1);
        return P1 + mu * (P2 - P1);
    }

    // atom function
    // set VBOs and generate mesh
    public void GenerateMesh()
    {
        if(mf.mesh == null)
        {
            lock (genMesh)
            {
                lock (calVert)
                {
                    Mesh mesh = new Mesh();
                    mf.mesh = mesh;
                    mesh.vertices = vertices.ToArray();
                    mesh.triangles = triangles.ToArray();
                    mesh.colors = colors.ToArray();

                    mesh.RecalculateNormals();
                    mesh.RecalculateBounds();
                    mesh.RecalculateTangents();
                    mesh.Optimize();
                    mesh.name = string.Format("Chunk({0},{1},{2})", xPos, yPos, zPos);
                    if (mesh.bounds.extents.magnitude > 0.5) mc.sharedMesh = mesh;
                    if (!MapManager.readyStatus) Interlocked.Increment(ref MapManager.readyCount);
                }
                
            }
        }
        else
        {
            lock(genMesh)
            {
                lock(calVert)
                {
                    mf.mesh.Clear(false);
                    mf.mesh.vertices = vertices.ToArray();
                    mf.mesh.triangles = triangles.ToArray();
                    mf.mesh.colors = colors.ToArray();

                    mf.mesh.RecalculateNormals();
                    mf.mesh.RecalculateBounds();
                    mf.mesh.RecalculateTangents();
                    mf.mesh.Optimize();
                    if (mf.mesh.bounds.extents.magnitude > 0.5) mc.sharedMesh = mf.mesh;
                    if (!MapManager.readyStatus) Interlocked.Increment(ref MapManager.readyCount);
                }
                
            }
        }
        
        
    }

}
