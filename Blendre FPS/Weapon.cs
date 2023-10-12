using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;
using Mirror;

public class Weapon : NetworkBehaviour
{
    public static Weapon Instance;

    #region Variables
    public Gun[] loadout;
    public GameObject[] fakegun_list;
    public Transform weapon_aim_position;
    public Transform weaponParent;
    public Transform fake_weapon_parent;
    public GameObject bulletholePrefeb;
    public GameObject shootEffectPrefeb;
    public LayerMask canBeShot;
    private float[] currentCooldown = new float[4];
    GameObject LDamage;
    GameObject RDamage;

    public Transform player_camera;
    public Transform left_hand;
    public GameObject currentWeapon;
    private GameObject currentfakeWeapon;
    public int current_weapon_index = 0;
    private int currentIndex = 0;
    private Quaternion weapon_parent_rotation;
    private Vector3 weapon_parent_position;
    private GameObject bullet_pack;
    public GameObject bullet_pack_prefeb;
    public bool is_bullet_pack = false;
    public GameObject sight;
    public bool is_Arming = false;
    private Vector3 shoot_position;
    public GameObject cross_hair_1;
    public GameObject cross_hair_2;
    private bool has_rotated_cross_hair = false;
    private Transform QK_position;
    public int damage_buff;
    public int damage_buff_parameter = 10;
    public int current_damage;
    public GameObject[] bullet_prefebs;
    #endregion
    // Start is called before the first frame update
    #region Monobehavior Callbacks
    void Start()
    {
        LDamage = GameObject.Find("BUFF_L").transform.Find("BUFF_L_Damage").gameObject;
        RDamage = GameObject.Find("BUFF_R").transform.Find("BUFF_R_Damage").gameObject;
        if (isLocalPlayer)
        {
            Instance = this;
            gameObject.GetComponent<motion>().enabled = true;
            gameObject.GetComponent<motion>().weapon = this;
        }
        Equip(0);
        shoot_position = weapon_aim_position.localPosition;

    }

    // implemented by my teammate
    // basically update weapon status from user input
    void Update()
    {
        ...
    } 

    private void Aim(bool p_isAming)
    {
        player_camera.GetComponent<Camera>().fieldOfView = 40;
    }

    void Shoot()
    {
        ...
    }


    IEnumerator TimeCount()
    {
        yield return new WaitForSeconds(0.6F);
        currentCooldown[0] = 0;
    }

    IEnumerator TimeCount1()
    {
        yield return new WaitForSeconds(0.6F);
        currentCooldown[1] = 0;
    }

    IEnumerator TimeCount2()
    {
        yield return new WaitForSeconds(5);
        currentCooldown[2] = 0;
    }

    // behaviors that are called from the server side
    [Command]
    public void BulletFire(int type, Vector3 pos, Vector3 dir, int damage, int player)
    {
        PlayerFire(type, pos, dir, damage, player);
    }

    [ClientRpc]
    public void PlayerFire(int type, Vector3 pos, Vector3 dir, int damage, int player)
    {
        Vector3 tar = pos + dir;
        GameObject bullet = Instantiate(bullet_prefebs[currentIndex], pos, Quaternion.identity);
        bullet.GetComponent<bullet_hit>().bulletType = type;
        bullet.GetComponent<bullet_hit>().damage = damage;
        bullet.GetComponent<bullet_hit>().player = player;
        bullet.transform.LookAt(tar);
        Rigidbody bullet_rig = bullet.GetComponent<Rigidbody>();
        bullet_rig.GetComponent<Rigidbody>().AddForce(dir.normalized * 50f);

        // type 2 bullet affect a large portion of the map
        // no need to detect collision with the ground
        // so calculate here
        if(type == 2)
        {
            Type2Bullet(pos, dir.normalized);
        }
    }

    void Type2Bullet(Vector3 pos, Vector3 dir)
    {
        float radius = 2.5f;
        List<Chunk> chunkBuffer = new List<Chunk>();
        Vector3 tmp;
        Chunk c;
        // traverse chunks to find the chunks that may 
        // interact with the bullet
        for (int w = 0; w < MapManager._instance.width; w++)
        {
            for (int h = 0; h < MapManager._instance.height; h++)
            {
                for (int l = 0; l < MapManager._instance.length; l++)
                {
                    c = MapManager.chunks[w, h, l];
                    tmp = c.chunkBase*0.5f + Vector3.one*radius - pos;
                    // early filter out
                    if (Vector3.Dot(tmp,dir.normalized) < -5) continue;
                    // send chunk to update buffer
                    if((tmp - Vector3.Project(tmp,dir)).magnitude < 10)
                    {
                        chunkBuffer.Add(c);
                    }
                }
            }
        }
        Debug.Log(chunkBuffer.Count);
        Vector3 current = Vector3.zero;
        Chunk[] carray = chunkBuffer.ToArray();
        // multi-threaded terrain modification
        for(int i = 0; i < carray.Length; i++)
        {
            HoleStruct hs = new HoleStruct{dir=dir,pos=pos,chunk=carray[i].chunkNum};
            ThreadPool.QueueUserWorkItem(new WaitCallback(HoleDrillThreading), hs);
        }
    }

    public struct HoleStruct
    {
        public Vector3 dir;
        public Vector3 pos;
        public Vector3Int chunk;
    }

    // this function generates a perpentrated hole in the map
    void HoleDrillThreading(object obj)
    {
        float radius = 2.5f;
        HoleStruct hs = (HoleStruct)obj;
        Vector3 dir = hs.dir;
        Vector3 pos = hs.pos;
        Chunk d = MapManager.chunks[hs.chunk.x, hs.chunk.y, hs.chunk.z];
        Vector3 current = Vector3.one;
        Vector3 grid;
        // search if each grid is affected
        for (int x = 0; x <= Chunk.chunkSize; x++)
        {
            current.x = x;
            for (int z = 0; z <= Chunk.chunkSize; z++)
            {
                current.z = z;
                for (int y = 0; y <= Chunk.chunkSize; y++)
                {
                    current.y = y;
                    Vector3 gridIndex = d.chunkBase + current;
                    grid = gridIndex * 0.5f - pos;
                    // early filter out
                    if (Vector3.Dot(grid, dir.normalized) < 0) continue;
                    // modify map
                    if ((grid - Vector3.Project(grid, dir.normalized)).magnitude <= radius)
                    {
                        try
                        {
                            MapManager.gridVal[(int)gridIndex.x, (int)gridIndex.y, (int)gridIndex.z] = 1;
                        }
                        catch { }
                    }
                }
            }
        }
        // recalculate mesh for affected chunks
        ThreadPool.QueueUserWorkItem(new WaitCallback(MapManager.chunks[d.chunkNum.x, d.chunkNum.y, d.chunkNum.z].CalculateVerts));
    }


    [Command]
    public void OnServerHitGround(int type, Vector3 pos)
    {
        OnClientHitGround(type, pos);
        if(MyNetworkManager._instance.mode == NetworkManagerMode.ServerOnly)
        {
            OnHitGround(type, pos);
        }
    }

    [ClientRpc]
    public void OnClientHitGround(int type, Vector3 pos)
    {
        OnHitGround(type, pos);
    }

    void OnHitGround(int type, Vector3 pos)
    {
        Vector3Int chunkBase = new Vector3Int((int)pos.x / 5, (int)pos.y / 5, (int)pos.z / 5);
        Vector3Int gridBase = new Vector3Int((int)(pos.x * 2), (int)(pos.y * 2), (int)(pos.z * 2));
        Vector3Int gridtemp = new Vector3Int();
        Debug.Log("Shoot at chunk: " + chunkBase.ToString() + " grid: " + gridBase.ToString() + " type: " + type.ToString());
        
        // type 0 generate a sphere hole on the map
        if (type == 0)
        {
            int radius = 5;
            for (int w = gridBase.x - radius; w <= gridBase.x + radius; w++)
            {
                if (w < 0 || w > MapManager.gridOnWidth) continue;
                gridtemp.x = w;
                for (int h = gridBase.y - radius; h <= gridBase.y + radius; h++)
                {
                    if (h < 0 || h > MapManager.gridOnHeight) continue;
                    gridtemp.y = h;
                    for (int l = gridBase.z - radius; l <= gridBase.z + radius; l++)
                    {
                        if (l < 0 || l > MapManager.gridOnLength) continue;
                        gridtemp.z = l;
                        float distance = Vector3Int.Distance(gridtemp, gridBase);
                        if (distance < radius)
                        {
                            try
                            {
                                MapManager.gridVal[w, h, l] = (1.0f - distance / radius);
                            }
                            catch
                            {
                                Debug.LogWarning("Grid Error on: " + gridtemp.ToString());
                            }
                        }
                    }
                }
            }
            for (int w = chunkBase.x - 1; w < chunkBase.x + 2; w++)
            {
                for (int h = chunkBase.y - 1; h < chunkBase.y + 2; h++)
                {
                    for (int l = chunkBase.z - 1; l < chunkBase.z + 2; l++)
                    {
                        try
                        {
                            ThreadPool.QueueUserWorkItem(new WaitCallback(MapManager.chunks[w, h, l].CalculateVerts));
                        }
                        catch
                        {
                            Debug.LogWarning(string.Format("Chunk Error on: ({0},{1},{2})",w,h,l));
                        }
                    }
                }
            }
        }
        // type 1 generate a sphere on the map
        else if (type == 1)
        {
            int radius = 5;
            Collider[] colliders = Physics.OverlapSphere(pos, radius / 2);
            foreach (Collider hit in colliders)
            {
                // push away player to avoid
                // the player being stucked in the generated ball
                if (hit.gameObject.tag == "Player")
                {
                    try
                    {
                        hit.attachedRigidbody.velocity = (QK_position.position - pos).normalized * 15;
                    }
                    catch { }

                }
            }
            for (int w = gridBase.x - radius; w <= gridBase.x + radius; w++)
            {
                if (w < 0 || w > MapManager.gridOnWidth) continue;
                gridtemp.x = w;
                for (int h = gridBase.y - radius; h <= gridBase.y + radius; h++)
                {
                    if (h < 0 || h > MapManager.gridOnHeight) continue;
                    gridtemp.y = h;
                    for (int l = gridBase.z - radius; l <= gridBase.z + radius; l++)
                    {
                        if (l < 0 || l > MapManager.gridOnLength) continue;
                        gridtemp.z = l;
                        float distance = Vector3Int.Distance(gridtemp, gridBase);
                        if (distance <= radius)
                        {
                            try
                            {
                                MapManager.gridVal[w, h, l] = distance / radius - 0.9f;
                            }
                            catch
                            {
                                Debug.LogWarning("Grid Error on: " + gridtemp.ToString());
                            }
                        }
                    }
                }
            }
            // recalculate chunk around the hit point
            for (int w = chunkBase.x - 1; w < chunkBase.x + 2; w++)
            {
                for (int h = chunkBase.y - 1; h < chunkBase.y + 2; h++)
                {
                    for (int l = chunkBase.z - 1; l < chunkBase.z + 2; l++)
                    {
                        try
                        {
                            ThreadPool.QueueUserWorkItem(new WaitCallback(MapManager.chunks[w, h, l].CalculateVerts));
                        }
                        catch
                        {
                            Debug.LogWarning(string.Format("Chunk Error on: ({0},{1},{2})",w,h,l));
                        }
                    }
                }
            }
        }
    }


    [Command]
    public void PlayerEatDamageBuff(int playerNum, NetworkIdentity item)
    {
        AddDamageBuff(playerNum);
        NetworkServer.Destroy(item.gameObject);
    }

    [ClientRpc]
    public void AddDamageBuff(int playerNum)
    {
        if (playerNum == 1)
        {
            LDamage.SetActive(true);
            LDamage.GetComponent<BUFF>().StartBuff(1);
        }
        else
        {
            RDamage.SetActive(true);
            RDamage.GetComponent<BUFF>().StartBuff(1);
        }
        if(playerNum == PlayerManager.localPlayer.localPlayerNum)
        {
            if(damage_buff == 0)
            {
                damage_buff++;
            }
        }

    }

    public void MinusDamageBuff()
    {
        if (damage_buff == 1)
        {
            damage_buff--;
        }
    }
    #endregion
}
