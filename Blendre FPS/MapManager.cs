using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Threading;
using UnityEngine;
using Mirror;

public class MapManager : MonoBehaviour
{
    public GameObject chunkPrefab;
    public Transform chunkHolder;
    public SimplexNoiseGenerator noiseGenrator;
    public Gradient grad;
    public static MapManager _instance;
    public static Vector3 seed;
    public static int mode = 0;
    public GameObject[] items;

    public static Chunk[,,] chunks;
    public GameObject[,,] chunkObjs;
    public static float[,,] gridVal;

    public int width;
    public int length;
    public int height;
    public static int gridOnWidth, gridOnHeight, gridOnLength;
    public static int readyCount;
    public static bool readyStatus = false;

    public static ConcurrentQueue<Chunk> refreshBuffer = new ConcurrentQueue<Chunk>();

    public static float noiseThreshold = 0f;
    // Start is called before the first frame update
    void Awake()
    {
        Debug.Log(_instance);
        _instance = this;
        seed = Random.Range(0, 1000) * Vector3.one;
        Chunk.gradient = grad;
        noiseGenrator = new SimplexNoiseGenerator();
        chunks = new Chunk[width, height, length];
        chunkObjs = new GameObject[width, height, length];
        ChunkInit();

        gridOnWidth = width * Chunk.chunkSize;
        gridOnHeight = height * Chunk.chunkSize;
        gridOnLength = length * Chunk.chunkSize;
        gridVal = new float[gridOnWidth, gridOnHeight, gridOnLength];
        Debug.Log("Map manager intialized");
    }

    public void MapInit(int r, int mode)
    {
        Chunk.mode = mode;
        seed = r * Vector3.one;
        readyCount = 0;
        for (int w = 0; w < width; w++)
        {
            for (int l = 0; l < length; l++)
            {
                for (int h = 0; h < height; h++)
                {
                    // initialize each chunk using threadpool
                    ThreadPool.QueueUserWorkItem(new WaitCallback(chunks[w, h, l].InitMap));
                }
            }
        }
        // wait until all chunks are initialized
        StartCoroutine(WaitInitReady());

    }

    IEnumerator WaitInitReady()
    {
        while(readyCount != width * height * length)
        {
            yield return null;
        }
        Debug.Log("Map init ready");
        readyCount = 0;
        GetMesh();
        int meshTime = 7;
        while (readyCount != width * height * length)
        {
            meshTime--;
            if (meshTime == 0) break;
            yield return new WaitForSeconds(1);
        }
        Debug.Log("Mesh init ready");
        readyStatus = true;
        if (MyNetworkManager._instance.mode == NetworkManagerMode.Host)
        {
            MyNetworkManager._instance.SceneReady += 1;
            MyNetworkManager._instance.MapReady();
        }
        else if (MyNetworkManager._instance.mode == NetworkManagerMode.ClientOnly) MyNetworkManager._instance.MapReady();
    }

    // update chunks from the refresh buffer
    private void LateUpdate()
    {
        Chunk c;
        for(int i = 0; i < 15; i++)
        {
            if (refreshBuffer.TryDequeue(out c))
            {
                c.GenerateMesh();
            }
            else break;
        }
    }

    public void GetMesh()
    {
        for (int w = 0; w < width; w++)
        {
            for (int l = 0; l < length; l++)
            {
                for (int h = 0; h < height; h++)
                {
                    ThreadPool.QueueUserWorkItem(new WaitCallback(chunks[w, h, l].CalculateVerts));
                }
            }
        }
    }

    // generate empty chunk objects
    void ChunkInit()
    {
        Vector3 current = Vector3.zero;
        for (int w = 0; w < width; w++)
        {
            current.x = w;
            for (int l = 0; l < length; l++)
            {
                current.z = l;
                for (int h = 0; h < height; h++)
                {
                    current.y = h;
                    GameObject obj = Instantiate(chunkPrefab, current * Chunk.chunkSize * Chunk.gridLength,
                        Quaternion.identity, chunkHolder);
                    obj.name = string.Format("Chunk({0},{1},{2})", w, h, l);
                    Chunk c = new Chunk(obj, w, h, l);
                    chunks[w, h, l] = c;
                    chunkObjs[w, h, l] = obj;
                }
            }
        }
        return;
    }

    [Server]
    public void ItemsInit()
    {
        Vector3 basePos;
        Vector2 pos = Vector3.zero;
        int index;
        float height;
        RaycastHit info;
        for(int x = 0; x < 5; x++)
        {
            for (int y = 0; y < 5; y++)
            {
                pos.x = Mathf.PerlinNoise(seed.x + x, seed.y);
                pos.y = Mathf.PerlinNoise(seed.x, seed.y + y);
                index = Mathf.FloorToInt((Noise.Simplex2D(new Vector2(seed.x + x, seed.y + y),1).value + 1) * items.Length * 0.5f);
                pos = pos * 15 + new Vector2(x*15,y*15);
                Physics.Raycast(new Vector3(pos.x, 40, pos.y), Vector3.down, out info, 40);
                height = Mathf.Max(info.point.y - 4, 3);
                basePos = new Vector3(pos.x, height, pos.y);
                GameObject item = Instantiate(items[index], basePos, Quaternion.identity);
                NetworkServer.Spawn(item);
            }
        }
    }
    
}
