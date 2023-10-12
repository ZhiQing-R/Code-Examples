#pragma once
#include "path_finder_base.hpp"
#include "external/gridmap/stamp.hpp"
#include "jps.hpp"
#include "external/utils/utility_helper.hpp"

#define MAX_NODES_PER_CHUNK 64
#define ISLAND_RAW 0xff
#define ISLAND_IMPASSIBLE 0xfe
#define ISLAND_START 0xfd

#define D  10
#define D2 14
#define D2D 4

namespace gridmap::path_finding {

    struct Boundary {
        IntVec2 box_min;
        IntVec2 box_max;
        Boundary() : box_min(0, 0), box_max(0, 0) {};
        Boundary(IntVec2 bmin, IntVec2 bmax) : box_min(bmin), box_max(bmax) {};
    };


    class HPA : public PathFinder {

    private:

        // nodes declaration
        struct AbstractNode;
        struct Edge;
        struct Chunk;
        struct HPANode;

        enum NodeType
        {
            RIGHT = 0,
            LEFT = 1,
            BOTTOM = 2,
            TOP = 3,
            TEMP = 4
        };

        enum class NodeStat
        {
            ACTIVE,
            DISABLE,
            MODIFIED,
        };

        struct AbstractNode {
            IntVec2 pos;
            IntVec2 cpos;
            uint8_t global_id = 0;
            uint8_t local_id = 0;
            NodeType type;
            std::list<Edge> edges;
            AbstractNode* pair;
            NodeStat stat = NodeStat::ACTIVE;
            explicit AbstractNode(IntVec2 pos, NodeType t = TEMP) : pos(pos), cpos(pos), type(t) {};
            AbstractNode() = default;
        };

        struct Edge {
            AbstractNode* end;
            unsigned int weight;
            Edge(AbstractNode* end, unsigned int weight)
                : end(end)
                , weight(weight) {};
        };

        struct Chunk
        {
            static int CHUNK_WIDTH;
            static int CHUNK_SIZE;
            std::vector<AbstractNode> nodes;
            std::unique_ptr<uint8_t[]> local_island;
            Boundary bound;
            int island_num = 0;
            bool dirty = false;

            Chunk(IntVec2 id) : local_island(std::make_unique<uint8_t[]>(CHUNK_SIZE))
            {
                bound = Boundary(IntVec2(id.x * CHUNK_WIDTH, id.y * CHUNK_WIDTH),
                    IntVec2(id.x * CHUNK_WIDTH + CHUNK_WIDTH - 1, id.y * CHUNK_WIDTH + CHUNK_WIDTH - 1));
                nodes.reserve(MAX_NODES_PER_CHUNK);
            };

        };

        // used in realtime path finding
        struct HPANode {
            HPANode()
                : index(0, 0)
                , G(INT32_MAX)
                , F(INT32_MAX)
                , parent(nullptr)
                , abs_node(nullptr)
            {};

            HPANode(const IntVec2& index, unsigned int G, unsigned int H, AbstractNode* abs_node)
                : index{ index }
                , G(G)
                , H(H)
                , F(G + H)
                , parent(nullptr)
                , abs_node(abs_node)
            {};

            unsigned int G;
            unsigned int H;
            unsigned int F;
            IntVec2 index;
            HPANode* parent;
            const AbstractNode* abs_node;

        };

        struct HPACache
        {
            HPACache(const GridStamp& stamp) : stamp(stamp) {};
            void reset();

            GridStamp stamp;
            std::vector<Chunk> chunks;
            std::unordered_set<int> dirty_chunk_ids;

            using node_map = std::unordered_multimap<IntVec2, AbstractNode*>;
            node_map nodes_table;
            std::unordered_set<IntVec2> blockers;

            bool ready = false;
        };

        enum class HPATaskType
        {
            CacheInit,
            ChunkUpdate,
        };

        struct HPATask
        {
            HPATask(HPATaskType type, HPACache* cache, Chunk* chunk) 
            : 
                type(type),
                cache(cache),
                chunk(chunk)
            {};
            HPATaskType type;
            HPACache* cache;
            Chunk* chunk;
            int progress = 0;
        };

    public:
        // core method
        bool check_stamp_at(const GridStamp& stamp, const IntVec2& index, const std::optional<IntVec2>& opt_ignored_index) const override;
        bool check_stamp_at(const GridStamp& stamp, const IntVec2& index) const;
        bool check_init_map(const GridStamp& stamp, const IntVec2& index) const;
        std::vector<IntVec2> find_path(const IntVec2& from_index, const IntVec2& to_index, const GridStamp& stamp) const override;
        std::vector<IntVec2> find_bounded_path(const IntVec2& from_index, const IntVec2& to_index, const GridStamp& stamp,
            const IntVec2& bound_center, const int& bound_size) const override;
        std::optional<unsigned> get_path_length(const IntVec2& from_index, const IntVec2& to_index, const GridStamp& stamp) const override;
        std::optional<unsigned> get_bounded_path_length(const IntVec2& from_index, const IntVec2& to_index, const GridStamp& stamp,
            const IntVec2& bound_center, const int& bound_size) const override;
        bool check_unstuck(const IntVec2& center_index, const int& check_size, const GridStamp& stamp) const override;
        fixmath_utility::ColorRGBA get_color(const uint8_t channel_id, const IntVec2& index) const override;
        std::string to_string() const override;

    protected:
        // signal
        void on_map_built(const int nx, const int ny, const uint8_t channel_count) override;
        void on_stamp_built(const GridStamp& stamp) override;
        void on_stamp_expired(const GridStamp& stamp) override;
        void on_channel_flushed(const GridChannel& channel) override;
        void on_grid_changed(const uint8_t& channel_id, const IntVec2& index, const GridData& grid) override;
        void on_frame_end() override;
        void on_frame_start() override;


    public:
        HPA() noexcept :
            jps_finder(std::make_unique<JPS>()),
            jps_init_finder(std::make_unique<JPS>())
        {}

        //after testing, the recommanded cluster size is width/16
        void build_abstract_map(const GridStamp& stamp, HPACache& cache);

    private:
        std::unordered_map<uint8_t, ChannelCPtr> channels;
        std::unordered_map<uint8_t, HPACache> caches;
        using GridPath = std::vector<IntVec2>;

        //Chunk attributes
        int width, height;
        int CHUNK_WIDTH = 0;
        int L1WIDTH = 0, L1HEIGHT = 0;
        std::unique_ptr<JPS> jps_finder;
        std::unique_ptr<JPS> jps_init_finder;

        std::queue<HPATask> task_queue;


        /***************************** Cache Build / Modification *****************************/

        //Update local island
        void update_local_island(Chunk& chunk, const GridStamp& stamp, bool init = false);
        void fill_local_island_id(uint8_t* island, IntVec2 pos, uint8_t id);

        //Build inter edges between two chunks
        //axis = true means horizontal edge
        void build_inter_edges(Chunk& c1, Chunk& c2, bool axis, const GridStamp& stamp);
        void build_inter_edges(Chunk& c1, Chunk& c2, bool axis, int pos, int line_size);

        //Build inter edges inside a margin
        void build_intra_edges(Chunk& chunk, const GridStamp& stamp);

        void find_entrance(const HPACache& cache, const GridStamp& stamp, AbstractNode& node);
        std::optional<IntVec2> find_entrance(const GridStamp& stamp, const AbstractNode& node, const uint8_t target) const;

        void on_frame_end(HPACache& cache);

        void process();
        bool process_chunk_update(HPATask& task, int& token);
        bool process_cache_init(HPATask& task, int& token);

        void update_node(const GridStamp& stamp, AbstractNode* node);

        /***************************** Runtime Utils *****************************/

        //Insert and remove the start / end point
        //If the node is already an abstract node, return false and the
        //reference to this abstract node
        //If it is a new node, return true and its reference
        std::pair<bool, AbstractNode&> gen_start_node(const IntVec2& node, const GridStamp& stamp, const HPACache& cache) const;
        std::unordered_map<IntVec2, unsigned int> gen_dest_map(uint8_t& gid, const IntVec2& node, const GridStamp& stamp, const HPACache& cache) const;

        //A checker utilize local island
        bool local_checker(const HPACache& cache, IntVec2 pos) const;
        bool local_checker(const Chunk& chunk, IntVec2 pos) const;

        //compare local island id between two index
        bool cmp_island_id(const Chunk& chunk, const IntVec2 n1, const IntVec2 n2) const;

        //get local island id
        uint8_t get_island_id(const Chunk& chunk, const IntVec2& n1) const;
        uint8_t get_island_id(const HPACache& cache, const IntVec2& n1) const;
        std::vector<uint8_t> get_island_id_around(const Chunk& chunk, const IntVec2& n1) const;

        //get chunk of a index
        const Chunk& get_chunk(const HPACache& cache, const IntVec2& node) const;

        //Evaluate if we can use a line to connect a and b
        inline bool connect_line(const HPACache& cache, const IntVec2 a, const IntVec2 b) const;

        static inline unsigned heuristic_distance(const IntVec2& a, const IntVec2& b) noexcept;
        class HPANodePtrComparison 
        {
        public:
            bool operator() (const HPANode* lhs, const HPANode* rhs) const noexcept
            {
                return (lhs->F > rhs->F);
            }
        };
        inline uint8_t stamp_mask(const GridStamp& stamp) const;

        /***************************** Debugging *****************************/

        mutable utility_helper::Timer timer;
        void print_local_island(const Chunk& chunk) const;

    };


};