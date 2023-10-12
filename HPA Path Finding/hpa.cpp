#pragma once
#include "hpa.hpp"
#include "external/gridmap/channel.hpp"

namespace gridmap::path_finding {

    const static std::array<IntVec2, 4> dirs = {
         IntVec2(1, 0),
         IntVec2(-1, 0),
         IntVec2(0, 1),
         IntVec2(0, -1)
    };

    inline unsigned HPA::heuristic_distance(const IntVec2& a, const IntVec2& b) noexcept
    {
        const auto dx = std::abs(b.x - a.x);
        const auto dy = std::abs(b.y - a.y);
        return D * std::max(dx, dy) + D2D * std::min(dx, dy);
    }

    bool HPA::check_stamp_at(const GridStamp& stamp, const IntVec2& index, const std::optional<IntVec2>& opt_ignored_index) const {
        return jps_finder->check_stamp_at(stamp, index, opt_ignored_index);
    }

    bool HPA::check_stamp_at(const GridStamp& stamp, const IntVec2& index) const {
        return jps_finder->check_stamp_at(stamp, index, {});
    }

    bool HPA::check_init_map(const GridStamp& stamp, const IntVec2& index) const {
        return jps_init_finder->check_stamp_at(stamp, index, {});
    }

    /* find path main funciton */
    std::vector<IntVec2> HPA::find_path(const IntVec2& from_index, const IntVec2& to_index, const GridStamp& stamp) const {
        ProfileTaskSection("HPA find path");
        timer.Mark();

        // use jps if no cache exist
        auto cache_pair = caches.find(stamp_mask(stamp));
        if (cache_pair == caches.end() || !cache_pair->second.ready)
            return jps_finder->find_path(from_index, to_index, stamp);
        const HPACache& cache = cache_pair->second;

        std::unordered_map<IntVec2, HPANode> nodes;
        nodes.reserve(cache.nodes_table.size() / 2);

        std::priority_queue<HPANode*, std::vector<HPANode*>, HPANodePtrComparison> open_list;

        std::pair<bool, AbstractNode&> pstart = gen_start_node(from_index, stamp, cache);
        AbstractNode& start = pstart.second;
        uint8_t gid;
        std::unordered_map<IntVec2, unsigned int> dests = gen_dest_map(gid, to_index, stamp, cache);
        if (start.edges.size() == 0 || dests.empty()) 
            return GridPath();
        if (start.global_id != gid) 
            return GridPath();

        // if two nodes are in the same chunk, try inter connect them
        if (pstart.first
            && from_index.x / CHUNK_WIDTH == to_index.x / CHUNK_WIDTH
            && from_index.y / CHUNK_WIDTH == to_index.y / CHUNK_WIDTH)
        {
            // add bound here
            const Chunk& chunk = get_chunk(cache, from_index);
            IntVec2 view_center = (chunk.bound.box_min + chunk.bound.box_max) / 2;
            unsigned view_bound = CHUNK_WIDTH / 2 + 1;
            auto p = jps_finder->get_bounded_path_length(from_index, to_index, stamp, view_center, view_bound);
            // if find a path, just return
            if (p.has_value())
            {
                return jps_finder->find_bounded_path(from_index, to_index, stamp, view_center, view_bound);
            }
        }

        // best node is the node nearest to destination
        // if no avaliable path, return the nearest node
        HPANode *node, *best_node;
        //insert the first node
        auto pair = &(nodes.emplace(from_index, HPANode(from_index, 0, heuristic_distance(from_index,to_index), &start)).first->second);
        open_list.push(pair);
        best_node = pair;

        // if the node is an abstract node, also add its pair node
        // into the open list
        if (pair->abs_node->type != TEMP)
        {
            IntVec2 pos = pair->abs_node->pair->cpos;
            auto p = &(nodes.emplace(pos,
                     HPANode(pos, D, heuristic_distance(pos, to_index),
                     pair->abs_node->pair)).first->second);
            p->parent = pair;
            open_list.push(p);
        }

        // store the nodes near the destination
        std::list<HPANode> tmp_pool;
        while (!open_list.empty())
        {
            // extract node from open list
            node = open_list.top();
            if (node->H < best_node->H)
                best_node = node;
            open_list.pop();

            if (node->index == to_index)
            {
                best_node = node;
                break;
            }

            // if find nodes around destination, add to open list
            if (dests.find(node->index) != dests.end())
            {
                unsigned int w = node->G + dests[node->index];
                HPANode* tmp_node = &(tmp_pool.emplace_back(
                    to_index, w, 0, nullptr));
                tmp_node->parent = node;
                open_list.push(tmp_node);
                continue;
            }

            const Chunk& chunk = get_chunk(cache, node->index);
            uint8_t id = node->abs_node->local_id;

            for (const auto& e : node->abs_node->edges)
            {
                if (e.end->stat == NodeStat::DISABLE) continue;
                if (id != ISLAND_START && id != e.end->local_id) continue;

                // calculate G and H
                AbstractNode* pair_node = e.end->pair;
                int pair_cost = node->G + e.weight + D;

                // check if we have visited this node
                auto try_find = nodes.find(pair_node->cpos);

                // if not, add this new node into the open list
                if (try_find == nodes.end())
                {
                    int pair_heuristic = heuristic_distance(pair_node->cpos, to_index);
                    auto next_node = &(nodes.emplace(pair_node->cpos,
                        HPANode(pair_node->cpos, pair_cost, pair_heuristic, pair_node)).first->second);
                    next_node->parent = node;
                    open_list.push(next_node);
                }
                // if yes and we find a shorter path to it, update it.
                else if (pair_cost < try_find->second.G)
                {
                    // here I simply add a new node to avoid heap update operation
                    int pair_heuristic = heuristic_distance(pair_node->cpos, to_index);
                    auto new_element = nodes.insert_or_assign(
                        pair_node->cpos, HPANode(pair_node->cpos, pair_cost, pair_heuristic, pair_node));
                    auto next_node = &(new_element.first->second);
                    next_node->parent = node;
                    open_list.push(next_node);
                }

            }
        }

        node = best_node;

        GridPath path, abs_path;
        GridPath tmp;
        unsigned view_bound = CHUNK_WIDTH / 2 + 2;
        // add abstract node
        while (node != nullptr)
        {
            abs_path.emplace_back(node->index);
            node = node->parent;
        }
        // find specific sub path between each pair of nodes
        for (auto it = abs_path.rbegin(); it != (abs_path.rend()-1); ++it)
        {
            const Chunk& chunk = get_chunk(cache, *it);
            IntVec2 view_center = (chunk.bound.box_min + chunk.bound.box_max) / 2;
            tmp = jps_finder->find_bounded_path(*it, *(it+1), stamp, view_center, view_bound);
            path.insert(path.end(), tmp.begin(), --tmp.end());
        }
        path.emplace_back(abs_path.front());

        //remove start node
        if (pstart.first)
            delete& (pstart.second);

        //return path;

        //smoothing
        GridPath rst;
        auto p1 = path.begin();
        auto p2 = p1 + 1;
        while (p2 != path.end())
        {
            rst.emplace_back(*p1);
            for (++p2; p2 != path.end(); ++p2)
            {
                if (!connect_line(cache, *p1, *p2))
                    break;
            }
            p1 = p2 - 1;
        }
        rst.emplace_back(*p1);
        
        return rst;
    }

    /***************************** Cache Build / Modification *****************************/

    int HPA::Chunk::CHUNK_WIDTH = 32;
    int HPA::Chunk::CHUNK_SIZE = 32*32;


    /* build cache */
    void HPA::build_abstract_map(const GridStamp& stamp, HPACache& cache) {
        timer.Mark();

        cache.chunks.clear();
        cache.nodes_table.clear();

        for (int j = 0; j < L1HEIGHT; ++j)
        {
            for (int i = 0; i < L1WIDTH; ++i)
            {
                auto& c = cache.chunks.emplace_back(IntVec2(i, j));
                update_local_island(c, stamp);
            }
        }
        std::cout << timer.Mark() << std::endl;
        // build inter edges
        for (int j = 0; j < L1HEIGHT; ++j)
        {
            for (int i = 0; i < L1WIDTH; ++i)
            {
                if (i + 1 < L1WIDTH)
                    build_inter_edges(cache.chunks[j * L1HEIGHT + i], cache.chunks[j * L1HEIGHT + i + 1], false, stamp);
                if (j + 1 < L1HEIGHT)
                    build_inter_edges(cache.chunks[j * L1HEIGHT + i], cache.chunks[j * L1HEIGHT + i + L1HEIGHT], true, stamp);
            }
        }
        std::cout << timer.Mark() << std::endl;
        // build intra edges
        for (auto& cn : cache.chunks)
        {
            for (auto& a : cn.nodes)
                a.local_id = get_island_id(cn, a.pos);
            build_intra_edges(cn, stamp);
        }
        std::cout << timer.Mark() << std::endl;
        // get global island id and add to hash map
        int id = 0;
        for (auto& cn : cache.chunks) {
            for (auto& a : cn.nodes) {
                cache.nodes_table.emplace(a.pos, &a);

                if (a.global_id > 0)
                    continue;

                ++id;
                a.global_id = id;
                AbstractNode* curr, * next;
                std::vector<AbstractNode*> queue;
                queue.emplace_back(&a);

                while (!queue.empty())
                {
                    curr = queue.back();
                    queue.pop_back();

                    for (auto& e : curr->edges)
                    {
                        next = e.end;
                        if (next->global_id == 0)
                        {
                            next->global_id = id;
                            queue.emplace_back(next);
                        }
                    }
                }

            }
        }
        std::cout << timer.Mark() << std::endl;
        cache.ready = true;
        std::cout << "Build abstract map done" << std::endl;
    }

    /* update local island */
    void HPA::update_local_island(Chunk& chunk, const GridStamp& stamp, bool init)
    {
        std::fill(chunk.local_island.get(),
            chunk.local_island.get() + Chunk::CHUNK_SIZE, ISLAND_RAW);

        if (init)
        {
            for (int y = 0; y < CHUNK_WIDTH; ++y)
            {
                for (int x = 0; x < CHUNK_WIDTH; ++x)
                {
                    if (!check_init_map(stamp, chunk.bound.box_min + IntVec2(x, y)))
                        chunk.local_island[y * CHUNK_WIDTH + x] = ISLAND_IMPASSIBLE;
                }
            }
        }
        else
        {
            for (int y = 0; y < CHUNK_WIDTH; ++y)
            {
                for (int x = 0; x < CHUNK_WIDTH; ++x)
                {
                    if (!check_stamp_at(stamp, chunk.bound.box_min + IntVec2(x, y)))
                        chunk.local_island[y * CHUNK_WIDTH + x] = ISLAND_IMPASSIBLE;
                }
            }
        }

        uint8_t id = 0;
        for (int y = 0; y < CHUNK_WIDTH; ++y)
        {
            for (int x = 0; x < CHUNK_WIDTH; ++x)
            {
                if (chunk.local_island[y * CHUNK_WIDTH + x] == ISLAND_RAW)
                {
                    fill_local_island_id(&chunk.local_island[0], IntVec2(x, y), id++);
                }
            }
        }
        chunk.island_num = id;
        
    }

    /* fill up island is with flood algorithm */
    void HPA::fill_local_island_id(uint8_t* island, IntVec2 pos, uint8_t id)
    {
        int idx = pos.y * CHUNK_WIDTH + pos.x;
        *(island + idx) = id;

        std::vector<IntVec2> queue;
        queue.reserve(Chunk::CHUNK_SIZE);
        queue.emplace_back(pos);

        IntVec2 curr, next;
        while (!queue.empty())
        {
            curr = queue.back();
            queue.pop_back();

            for (auto vec : dirs)
            {
                next = curr + vec;
                idx = next.y * CHUNK_WIDTH + next.x;

                if (next.x >= 0 && next.y >= 0 && next.x < CHUNK_WIDTH && next.y < CHUNK_WIDTH && *(island + idx) == ISLAND_RAW)
                {
                    *(island + idx) = id;
                    queue.emplace_back(next);
                }
            }
        }
    }

    /* find nodes that connect two chunks */
    void HPA::build_inter_edges(Chunk& c1, Chunk& c2, bool axis, const GridStamp& stamp)
    {
        int line_size = 0;
        if (axis)
        {
            int x = c1.bound.box_min.x;
            int y = c1.bound.box_max.y;
            int i;
            for (i = 0; i < CHUNK_WIDTH; ++i)
            {
                if (check_init_map(stamp, IntVec2(x + i, y)) &&
                    check_init_map(stamp, IntVec2(x + i, y + 1)))
                    ++line_size;
                else
                {
                    build_inter_edges(c1, c2, axis, i, line_size);
                    line_size = 0;
                }
            }
            build_inter_edges(c1, c2, axis, i, line_size);
        }
        else
        {
            int x = c1.bound.box_max.x;
            int y = c1.bound.box_min.y;
            int i;
            for (i = 0; i < CHUNK_WIDTH; ++i) {
                if (check_init_map(stamp, IntVec2(x, y + i)) &&
                    check_init_map(stamp, IntVec2(x + 1, y + i)))
                    ++line_size;
                else {
                    build_inter_edges(c1, c2, axis, i, line_size);
                    line_size = 0;
                }
            }
            build_inter_edges(c1, c2, axis, i, line_size);
        }
    }

    /* establish connection between two chunks */
    void HPA::build_inter_edges(Chunk& c1, Chunk& c2, bool axis, int i, int line_size)
    {
        if (line_size == 0) return;
        if (axis)
        {
            int x = c1.bound.box_min.x;
            int y = c1.bound.box_max.y;

            if (line_size < 6)
            {
                AbstractNode& a1 = c1.nodes.emplace_back(IntVec2(x + i - line_size / 2 - 1, y), NodeType::BOTTOM);
                AbstractNode& a2 = c2.nodes.emplace_back(IntVec2(x + i - line_size / 2 - 1, y + 1), NodeType::TOP);
                a1.edges.emplace_back(&a2, D);
                a2.edges.emplace_back(&a1, D);
                a1.pair = &a2;
                a2.pair = &a1;
            }
            else
            {
                AbstractNode& a1 = c1.nodes.emplace_back(IntVec2(x + i - line_size, y), NodeType::BOTTOM);
                AbstractNode& a2 = c2.nodes.emplace_back(IntVec2(x + i - line_size, y + 1), NodeType::TOP);
                a1.edges.emplace_back(&a2, D);
                a2.edges.emplace_back(&a1, D);
                a1.pair = &a2;
                a2.pair = &a1;

                AbstractNode& b1 = c1.nodes.emplace_back(IntVec2(x + i - 1, y), NodeType::BOTTOM);
                AbstractNode& b2 = c2.nodes.emplace_back(IntVec2(x + i - 1, y + 1), NodeType::TOP);
                b1.edges.emplace_back(&b2, D);
                b2.edges.emplace_back(&b1, D);
                b1.pair = &b2;
                b2.pair = &b1;
            }
        }
        else
        {
            int x = c1.bound.box_max.x;
            int y = c1.bound.box_min.y;

            if (line_size < 6)
            {
                AbstractNode& a1 = c1.nodes.emplace_back(IntVec2(x, y + i - line_size / 2 - 1), NodeType::RIGHT);
                AbstractNode& a2 = c2.nodes.emplace_back(IntVec2(x + 1, y + i - line_size / 2 - 1), NodeType::LEFT);
                a1.edges.emplace_back(&a2, D);
                a2.edges.emplace_back(&a1, D);
                a1.pair = &a2;
                a2.pair = &a1;
            }
            else
            {
                AbstractNode& a1 = c1.nodes.emplace_back(IntVec2(x, y + i - line_size), NodeType::RIGHT);
                AbstractNode& a2 = c2.nodes.emplace_back(IntVec2(x + 1, y + i - line_size), NodeType::LEFT);
                a1.edges.emplace_back(&a2, D);
                a2.edges.emplace_back(&a1, D);
                a1.pair = &a2;
                a2.pair = &a1;

                AbstractNode& b1 = c1.nodes.emplace_back(IntVec2(x, y + i - 1), NodeType::RIGHT);
                AbstractNode& b2 = c2.nodes.emplace_back(IntVec2(x + 1, y + i - 1), NodeType::LEFT);
                b1.edges.emplace_back(&b2, D);
                b2.edges.emplace_back(&b1, D);
                b1.pair = &b2;
                b2.pair = &b1;
            }
        }
    }

    /* find path weight for each pair of nodes within a chunk */
    void HPA::build_intra_edges(Chunk& chunk, const GridStamp& stamp)
    {
        if (chunk.nodes.size() < 2) return;

        IntVec2 view_center = (chunk.bound.box_min + chunk.bound.box_max) / 2;
        const unsigned view_bound = CHUNK_WIDTH / 2 + 1;

        for (auto i = chunk.nodes.begin(); i != chunk.nodes.end(); ++i)
        {
            auto j = i;
            for (++j; j != chunk.nodes.end(); ++j)
            {
                if (i->local_id != j->local_id)
                    continue;

                auto p = jps_init_finder->get_bounded_path_length(i->pos, j->pos, stamp, view_center, view_bound);
                if (p.has_value())
                {
                    i->edges.emplace_back(&(*j), p.value());
                    j->edges.emplace_back(&(*i), p.value());
                }
            }
        }
    }

    /* Deprecated */
    void HPA::update_node(const GridStamp& stamp, AbstractNode* node)
    {
        /*if (node->stat == NodeStat::ACTIVE)
        {
            if (!check_stamp_at(stamp, node->pos))
            {
                find_entrance(stamp, *(node));
                return;
            }
        }
        else
        {
            if (check_stamp_at(stamp, node->pos))
            {
                node->stat = NodeStat::ACTIVE;
                node->cpos = node->pos;
            }
        }*/
    }


    /***************************** Runtime Utils *****************************/

    /* compair local id of 2 nodes */
    bool HPA::cmp_island_id(const Chunk& chunk, const IntVec2 n1, const IntVec2 n2) const
    {
        IntVec2 vec1 = n1 - chunk.bound.box_min;
        IntVec2 vec2 = n2 - chunk.bound.box_min;

        int idx1 = vec1.y * CHUNK_WIDTH + vec1.x;
        int idx2 = vec2.y * CHUNK_WIDTH + vec2.x;

        if (idx1 >= 0 && idx2 >= 0 && idx1 < Chunk::CHUNK_SIZE && idx2 < Chunk::CHUNK_SIZE)
        {
            return chunk.local_island[idx1] == chunk.local_island[idx2];
        }

        return false;
    }

    /* get local id for a position*/
    uint8_t HPA::get_island_id(const Chunk& chunk, const IntVec2& n1) const
    {
        IntVec2 vec1 = n1 - chunk.bound.box_min;
        int idx = vec1.y * CHUNK_WIDTH + vec1.x;
        return chunk.local_island[idx];
    }

    /* 
    *   get all local ids around a node 
    *   used when need to find the local
    *   ids of a blocking identity
    */
    std::vector<uint8_t> HPA::get_island_id_around(const Chunk& chunk, const IntVec2& n1) const
    {
        std::vector<uint8_t> ids;
        IntVec2 v = n1 - chunk.bound.box_min, next;
        int idx;
        uint8_t id;
        for (auto vec : dirs)
        {
            next = v + vec;
            idx = next.y * CHUNK_WIDTH + next.x;
            if (next.x >= 0 && next.y >= 0 && next.x < CHUNK_WIDTH && next.y < CHUNK_WIDTH 
                && chunk.local_island[idx] != ISLAND_IMPASSIBLE)
                ids.push_back(chunk.local_island[idx]);
        }
        return ids;
    }

    /* get local island id */
    uint8_t HPA::get_island_id(const HPACache& cache, const IntVec2& n1) const
    {
        const Chunk& chunk = get_chunk(cache, n1);
        IntVec2 vec = n1 - chunk.bound.box_min;
        int idx1 = vec.y * CHUNK_WIDTH + vec.x;
        return chunk.local_island[idx1];
    }

    /* get reference to the chunk containing this node */
    const HPA::Chunk& HPA::get_chunk(const HPACache& cache, const IntVec2& node) const
    {
        int x = node.x / CHUNK_WIDTH;
        int y = node.y / CHUNK_WIDTH;
        return cache.chunks[y * L1WIDTH + x];
    }

    /* get start node */
    std::pair<bool, HPA::AbstractNode&> HPA::gen_start_node(const IntVec2& node, const GridStamp& stamp, const HPACache& cache) const
    {
        // if it is a existing abstract node, return its reference
        auto n = cache.nodes_table.find(node);
        if (n != cache.nodes_table.end())
            return std::pair<bool, AbstractNode&>(false, *(n->second));

        const Chunk& chunk = get_chunk(cache, node);
        //print_local_island(chunk);

        // set bound
        IntVec2 view_center = (chunk.bound.box_min + chunk.bound.box_max) / 2;
        int view_bound = CHUNK_WIDTH / 2 + 1;

        // get node info
        AbstractNode* new_node = new AbstractNode(node);
        auto ids = get_island_id_around(chunk, node);
        new_node->local_id = ISLAND_START;

        // check abstract node walkable
        for (const auto& n : chunk.nodes)
        {
            if (n.stat != NodeStat::DISABLE && std::find(ids.begin(), ids.end(), n.local_id) != ids.end())
            {
                // insert node use heuristic function
                //new_node->edges.emplace_back(const_cast<AbstractNode*>(&n), heuristic_distance(node,n.cpos));
                
                auto p = jps_finder->get_bounded_path_length(node, n.cpos, stamp, view_center, view_bound);
                if (p.has_value())
                    new_node->edges.emplace_back(const_cast<AbstractNode*>(&n), p.value());
                
            }
        }
        if (!new_node->edges.empty())
            new_node->global_id = new_node->edges.front().end->global_id;
        return std::pair<bool, AbstractNode&>(true, *new_node);

    }

    /* get hash map of nodes connected to the destination */
    std::unordered_map<IntVec2, unsigned int> HPA::gen_dest_map(uint8_t& gid, const IntVec2& node, const GridStamp& stamp, const HPACache& cache) const
    {
        const Chunk& chunk = get_chunk(cache, node);
        auto ids = get_island_id_around(chunk, node);
        std::unordered_map<IntVec2, unsigned int> ret;

        // case when destination is an abstract node
        if (cache.nodes_table.find(node) != cache.nodes_table.end())
        {
            const auto& n = *cache.nodes_table.find(node)->second;
            gid = n.global_id;
            ret.emplace(n.cpos, 0);
            ret.emplace(n.pair->cpos, D);
            for (const auto& e : n.edges)
            {
                if (e.end->stat != NodeStat::DISABLE && std::find(ids.begin(), ids.end(), e.end->local_id) != ids.end())
                    ret.emplace(e.end->cpos, e.weight);
            }
            return ret;
        }
        else
        {
            IntVec2 view_center = (chunk.bound.box_min + chunk.bound.box_max) / 2;
            int view_bound = CHUNK_WIDTH / 2 + 1;

            for (const auto& n : chunk.nodes)
            {
                if (n.stat != NodeStat::DISABLE && std::find(ids.begin(), ids.end(), n.local_id) != ids.end())
                {
                    //gid = n.global_id;
                    //ret.emplace(n.cpos, heuristic_distance(node,n.cpos));
                    
                    auto p = jps_finder->get_bounded_path_length(node, n.cpos, stamp, view_center, view_bound);
                    if (p.has_value())
                    {
                        gid = n.global_id;
                        ret.emplace(n.cpos, p.value());
                    }
                    
                }
            }

            return ret;
        }
    }

    /* check if we can reach from a to b use a straight line */
    bool HPA::connect_line(const HPACache& cache, const IntVec2 a, const IntVec2 b) const
    {

        int dx = abs(b.x - a.x);
        int dy = abs(b.y - a.y);
        int x = a.x;
        int y = a.y;
        int n = 1 + dx + dy;
        int x_inc = (b.x > a.x) ? 1 : -1;
        int y_inc = (b.y > a.y) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        // run one iteration ahead to ignore a's condition
        // this is necessary since "a" may be the identity
        // block this position
        if (error > 0)
        {
            x += x_inc;
            error -= dy;
        }
        else
        {
            y += y_inc;
            error += dx;
        }
        --n;
        for (; n > 0; --n)
        {
            if (!local_checker(cache, IntVec2(x, y))) return false;

            if (error > 0)
            {
                x += x_inc;
                error -= dy;
            }
            else
            {
                y += y_inc;
                error += dx;
            }
        }
        return true;
    }

    /* the checker utilize local island cache */
    bool HPA::local_checker(const HPACache& cache, IntVec2 pos) const
    {
        int x = pos.x / CHUNK_WIDTH;
        int y = pos.y / CHUNK_WIDTH;
        const Chunk& chunk = cache.chunks[y * L1WIDTH + x];
        const IntVec2 vec = pos - chunk.bound.box_min;
        return chunk.local_island[vec.y * CHUNK_WIDTH + vec.x] != ISLAND_IMPASSIBLE;
    }

    /* the checker utilize local island cache */
    bool HPA::local_checker(const Chunk& chunk, IntVec2 pos) const
    {
        const IntVec2 vec = pos - chunk.bound.box_min;
        return chunk.local_island[vec.y * CHUNK_WIDTH + vec.x] != ISLAND_IMPASSIBLE;
    }

    /* when abstract node is blocked, use it to find a subtitution position */
    void HPA::find_entrance(const HPACache& cache, const GridStamp& stamp, AbstractNode& node)
    {
        AbstractNode& pnode = *node.pair;
        IntVec2 tmp = node.pos;
        IntVec2 pair = pnode.pos;
        const IntVec2 dir = node.type < 2 ? dirs.at(2) : dirs.at(0);
        const Chunk& chunk = get_chunk(cache, tmp);
        const Chunk& pchunk = get_chunk(cache, pair);
        bool ncollide = true;
        while (get_island_id(chunk,tmp) == ISLAND_IMPASSIBLE || get_island_id(pchunk, pair) == ISLAND_IMPASSIBLE)
        {
            tmp += dir;
            pair += dir;
            if (tmp.x > chunk.bound.box_max.x || tmp.y > chunk.bound.box_max.y)
            {
                ncollide = false;
                break;
            }
            if (cache.nodes_table.find(tmp) != cache.nodes_table.end() 
                || cache.nodes_table.find(pair) != cache.nodes_table.end())
            {
                ncollide = false;
                break;
            }
        }
        if (ncollide &&
            get_island_id(chunk, tmp) != ISLAND_IMPASSIBLE && get_island_id(pchunk, pair) != ISLAND_IMPASSIBLE)
        {
            node.cpos = tmp;
            node.local_id = get_island_id(chunk, tmp);
            node.stat = NodeStat::MODIFIED;

            pnode.cpos = pair;
            pnode.local_id = get_island_id(pchunk, pair);
            pnode.stat = NodeStat::MODIFIED;
            return;
        }

        tmp = node.pos;
        pair = pnode.pos;
        ncollide = true;
        while (get_island_id(chunk, tmp) == ISLAND_IMPASSIBLE || get_island_id(pchunk, pair) == ISLAND_IMPASSIBLE)
        {
            tmp -= dir;
            pair -= dir;
            if (tmp.x < chunk.bound.box_min.x || tmp.y < chunk.bound.box_min.y)
            {
                ncollide = false;
                break;
            }
            if (cache.nodes_table.find(tmp) != cache.nodes_table.end()
                || cache.nodes_table.find(pair) != cache.nodes_table.end())
            {
                ncollide = false;
                break;
            }
        }
        if (ncollide &&
            get_island_id(chunk, tmp) != ISLAND_IMPASSIBLE && get_island_id(pchunk, pair) != ISLAND_IMPASSIBLE)
        {
            node.cpos = tmp;
            node.local_id = get_island_id(chunk, tmp);
            node.stat = NodeStat::MODIFIED;

            pnode.cpos = pair;
            pnode.local_id = get_island_id(pchunk, pair);
            pnode.stat = NodeStat::MODIFIED;
            return;
        }

        node.stat = NodeStat::DISABLE;
        pnode.stat = NodeStat::DISABLE;
    }

    void HPA::process()
    {
        //timer.Mark();
        int token = 100;
        bool done = true;
        while (!task_queue.empty() && token > 0)
        {
            HPATask& task = task_queue.front();
            
            switch (task.type)
            {
            case HPATaskType::ChunkUpdate:
                done = process_chunk_update(task, token);
                //printf("process chunk update: %d\n",token);
                break;
            case HPATaskType::CacheInit:
                done = process_cache_init(task, token);
                //printf("process cache init: %d\n", token);
                break;
            default:
                break;
            }
            if (done) task_queue.pop();
        }
        //if (token < 100)
        //    std::cout << timer.Mark() << std::endl;

    }

    bool HPA::process_chunk_update(HPATask& task, int& token)
    {
        HPACache& cache = *task.cache;
        Chunk& chunk = *task.chunk;
        update_local_island(chunk, cache.stamp);
        for (auto& n : chunk.nodes)
        {
            uint8_t id = get_island_id(chunk, n.pos);
            uint8_t pid = get_island_id(cache, n.pair->pos);
            if (id != ISLAND_IMPASSIBLE && pid != ISLAND_IMPASSIBLE)
            {
                n.local_id = id;
                n.cpos = n.pos;
                n.stat = NodeStat::ACTIVE;
                n.pair->local_id = pid;
                n.pair->cpos = n.pair->pos;
                n.pair->stat = NodeStat::ACTIVE;
            }
            else
                find_entrance(cache, cache.stamp, n);
        }
        chunk.dirty = false;
        --token;
        return true;
    }

    bool HPA::process_cache_init(HPATask& task, int& token)
    {
        HPACache& cache = *task.cache;
        const int cost = std::max(1,(CHUNK_WIDTH >> 5) * (CHUNK_WIDTH >> 5));

        while (task.progress < L1HEIGHT * L1WIDTH)
        {
            for (int i = 0; i < L1WIDTH; ++i)
            {
                update_local_island(cache.chunks[task.progress++], cache.stamp, true);
            }
            token -= L1WIDTH*cost;
            if (token <= 0) return false;
        }
        
        while (task.progress < 2 * L1HEIGHT * L1WIDTH)
        {
            int j = (task.progress - L1HEIGHT * L1WIDTH) / L1HEIGHT;
            for (int i = 0; i < L1WIDTH; ++i)
            {
                if (i + 1 < L1WIDTH)
                    build_inter_edges(cache.chunks[j * L1HEIGHT + i], cache.chunks[j * L1HEIGHT + i + 1], false, cache.stamp);
                if (j + 1 < L1HEIGHT)
                    build_inter_edges(cache.chunks[j * L1HEIGHT + i], cache.chunks[j * L1HEIGHT + i + L1HEIGHT], true, cache.stamp);
            }
            task.progress += L1WIDTH;
            token -= (L1WIDTH/10 + 1) * cost;
            if (token <= 0) return false;
        }

        while (task.progress < 3 * L1HEIGHT * L1WIDTH)
        {
            Chunk& cn = cache.chunks[task.progress - 2 * L1HEIGHT * L1WIDTH];
            for (auto& a : cn.nodes)
                a.local_id = get_island_id(cn, a.pos);
            build_intra_edges(cn, cache.stamp);
            ++task.progress;
            token -= 15 * cost;
            if (token <= 0) return false;
        }

        int id = 0;
        for (auto& cn : cache.chunks) {
            for (auto& a : cn.nodes) {
                cache.nodes_table.emplace(a.pos, &a);

                if (a.global_id > 0)
                    continue;

                ++id;
                a.global_id = id;
                AbstractNode* curr, * next;
                std::vector<AbstractNode*> queue;
                queue.emplace_back(&a);

                while (!queue.empty())
                {
                    curr = queue.back();
                    queue.pop_back();

                    for (auto& e : curr->edges)
                    {
                        next = e.end;
                        if (next->global_id == 0)
                        {
                            next->global_id = id;
                            queue.emplace_back(next);
                        }
                    }
                }
            }
        }
        token -= 50 * cost;
        cache.ready = true;
        std::cout << "Build abstract map done" << std::endl;
        return true;

    }

    uint8_t HPA::stamp_mask(const GridStamp& stamp) const
    {
        return (stamp.get_uniform_limitation() & 0x3f);
    }

    /* Deprecated */
    std::optional<IntVec2> HPA::find_entrance(const GridStamp& stamp, const AbstractNode& node, const uint8_t target) const
    {
        /*IntVec2 tmp = node.pos;
        IntVec2 pair = tmp + dirs[node.type];
        IntVec2 dir = node.type < 2 ? dirs[0] : dirs[2];
        int x = tmp.x / CHUNK_WIDTH;
        int y = tmp.y / CHUNK_WIDTH;
        const Chunk& chunk = chunks[y * L1HEIGHT + x];
        while (blockers.find(tmp) != blockers.end() || blockers.find(pair) != blockers.end())
        {
            tmp += dir;
            pair += dir;
            if (tmp.x >= chunk.bound.box_max.x || tmp.y >= chunk.bound.box_max.y)
                break;
        }
        
        IntVec2 vec = tmp - chunk.bound.box_min;
        if (chunk.local_island[vec.y * CHUNK_WIDTH + vec.x] == target && check_stamp_at(stamp, pair))
            return tmp;

        tmp = node.pos;
        pair = tmp + dirs[node.type];
        while (blockers.find(tmp) != blockers.end() || blockers.find(pair) != blockers.end())
        {
            tmp -= dir;
            pair -= dir;
            if (tmp.x <= chunk.bound.box_min.x || tmp.y <= chunk.bound.box_min.y)
                break;
        }
        
        vec = tmp - chunk.bound.box_min;
        if (chunk.local_island[vec.y * CHUNK_WIDTH + vec.x] == target && check_stamp_at(stamp, pair))
            return tmp;
        */
        return {};
    }



    /***************************** Callbacks *****************************/

    void HPA::on_map_built(const int nx, const int ny, const uint8_t channel_count)
    {
        jps_finder->on_map_built(nx, ny, channel_count);
        jps_init_finder->on_map_built(nx, ny, channel_count);
        width = nx;
        height = ny;
        //Chunk::CHUNK_WIDTH = std::max(nx, ny) / 16;
        Chunk::CHUNK_WIDTH = 64;
        Chunk::CHUNK_SIZE = Chunk::CHUNK_WIDTH * Chunk::CHUNK_WIDTH;
        CHUNK_WIDTH = Chunk::CHUNK_WIDTH;
        L1WIDTH = nx / CHUNK_WIDTH;
        L1HEIGHT = ny / CHUNK_WIDTH;
        return;
    }

    void HPA::on_grid_changed(const uint8_t& channel_id, const IntVec2& index, const GridData& grid) 
    {
        jps_finder->on_grid_changed(channel_id, index, grid);

        for (auto& p : caches)
        {
            HPACache& cache = p.second;
            if (cache.stamp.channel_id != channel_id) continue;

            int x = index.x / CHUNK_WIDTH;
            int y = index.y / CHUNK_WIDTH;
            int idx = y * L1WIDTH + x;
            Chunk& chunk = cache.chunks[idx];

            if (!cache.ready)
            {
                if (!chunk.dirty) 
                {
                    chunk.dirty = true;
                    task_queue.push(HPATask(HPATaskType::ChunkUpdate, &cache, &chunk));
                }
                continue;
            }

            //bool new_stat = channels.at(cache.stamp.channel_id)->check_stamp_at(cache.stamp,index);
            bool new_stat = ((grid.get_uniform_requirement() & cache.stamp.get_uniform_limitation()) == 0U);
            if (new_stat ^ local_checker(chunk, index))
            {
                if (!chunk.dirty)
                {
                    cache.dirty_chunk_ids.emplace(idx);
                    chunk.dirty = true;
                    task_queue.push(HPATask(HPATaskType::ChunkUpdate, &cache, &chunk));
                }
            }
        }
    }

    void HPA::on_frame_end()
    {
        jps_finder->on_frame_end();
        if (caches.size() == 0) return;
        process();
        /*for (auto& p : caches)
        {
            HPACache& cache = p.second;
            if (!cache.ready) continue;
            on_frame_end(cache);
        }*/
    }

    void HPA::on_frame_end(HPACache& cache)
    {
        const auto& stamp = cache.stamp;

        // update local islands of all dirty chunks
        for (auto idx : cache.dirty_chunk_ids)
        {
            update_local_island(cache.chunks[idx], stamp);
        }

        // check if abtract nodes is blocked
        // if yes, modify the position
        for (auto idx : cache.dirty_chunk_ids)
        {
            Chunk& chunk = cache.chunks[idx];
            for (auto& n : chunk.nodes)
            {
                uint8_t id = get_island_id(chunk, n.pos);
                uint8_t pid = get_island_id(cache, n.pair->pos);
                if (id != ISLAND_IMPASSIBLE && pid != ISLAND_IMPASSIBLE)
                {
                    n.local_id = id;
                    n.cpos = n.pos;
                    n.stat = NodeStat::ACTIVE;
                    n.pair->local_id = pid;
                    n.pair->cpos = n.pair->pos;
                    n.pair->stat = NodeStat::ACTIVE;
                }
                else
                    find_entrance(cache, stamp, n);
            }
            chunk.dirty = false;
        }
        cache.dirty_chunk_ids.clear();
    }

    void HPA::on_stamp_built(const GridStamp& stamp) 
    {
        if (stamp.size > 1) return;
        jps_finder->on_stamp_built(stamp);
        if (!stamp.ignore_unstable) return;

        auto channel_ptr = channels.find(stamp.channel_id);
        if (channel_ptr != channels.end() && stamp.size == 1)
        {
            HPACache& cache = caches.try_emplace(stamp_mask(stamp), HPACache(stamp)).first->second;
            for (int j = 0; j < L1HEIGHT; ++j)
                for (int i = 0; i < L1WIDTH; ++i)
                    cache.chunks.emplace_back(IntVec2(i, j));
            task_queue.push(HPATask(HPATaskType::CacheInit, &cache, nullptr));
            //build_abstract_map(stamp, cache);
        }
    }

    void HPA::on_channel_flushed(const GridChannel& channel)
    {
        jps_finder->on_channel_flushed(channel);
        jps_init_finder->on_channel_flushed(channel);
        channels.try_emplace(channel.get_channel_id(), channel.shared_from_this());
        /*auto tmp = channels_data.try_emplace(channel.get_channel_id(), std::vector<uniform_prop_bit_type>());
        if (tmp.second)
        {
            tmp.first->second.reserve(channel.get_data().size());
            for (int i = 0; i < channel.get_data().size(); ++i)
                tmp.first->second.emplace_back(channel.get_data()[i].get_uniform_requirement());
        }*/
        
        
        for (auto& p : caches)
        {
            HPACache& cache = p.second;
            cache.reset();
            for (int j = 0; j < L1HEIGHT; ++j)
                for (int i = 0; i < L1WIDTH; ++i)
                    cache.chunks.emplace_back(IntVec2(i, j));
            //build_abstract_map(cache.stamp, cache);
        }

    }

    void HPA::on_frame_start() 
    {
        jps_finder->on_frame_start();
    }

    void HPA::on_stamp_expired(const GridStamp& stamp) 
    {
        jps_finder->on_stamp_expired(stamp);
    }

    /***************************** Debugging *****************************/

    /* print local island in cosole (for debugging) */
    void HPA::print_local_island(const Chunk& chunk) const
    {
        std::cout << '(' << chunk.bound.box_min.x << ',' << chunk.bound.box_min.y << ')';
        for (int i = 0; i < Chunk::CHUNK_SIZE; ++i)
        {
            if (i % CHUNK_WIDTH == 0) std::cout << std::endl;
            if (chunk.local_island[i] == ISLAND_IMPASSIBLE)
                std::cout << '@';
            else
                std::cout << (int)chunk.local_island[i];
        }
        std::cout << std::endl;
        std::cout << std::endl;
    }

    fixmath_utility::ColorRGBA HPA::get_color(const uint8_t channel_id, const IntVec2& index) const
    {
        if (caches.size() == 0) return {};

        const HPACache& cache = caches.begin()->second;
        if (!cache.ready) return {};
        const Chunk& chunk = get_chunk(cache,index);
        IntVec2 vec = index - chunk.bound.box_min;

        //check if on abs node
        for (const auto& n : chunk.nodes)
        {
            if (n.stat != NodeStat::DISABLE && n.cpos == index) return fixmath_utility::ColorRGBA(250,50,0,200);
        }

        uint8_t id = get_island_id(chunk, index);
        if (id == ISLAND_IMPASSIBLE) return fixmath_utility::ColorRGBA(0, 0, 200, 20);

        if (index.x % CHUNK_WIDTH == 0 || index.y % CHUNK_WIDTH == 0
            || (index.x + 1) % CHUNK_WIDTH == 0 || (index.y + 1) % CHUNK_WIDTH == 0)
            return fixmath_utility::ColorRGBA(250, 200, 0, 20);

        else return fixmath_utility::ColorRGBA(250, 250 - id * 50, 250 - id * 50, 20);
    }

    std::string HPA::to_string() const
    {
        std::ostringstream oss;
        oss << "PathFinder[HPA]: Raw HPA path finder.\n" << "Chunk Width: " << CHUNK_WIDTH <<
            ". Stamp Num: " << caches.size() << ".\n";
        return oss.str();
    }

    /***************************** HPA Cache *****************************/

    void HPA::HPACache::reset()
    {
        ready = false;
        chunks.clear();
        dirty_chunk_ids.clear();
        nodes_table.clear();
        blockers.clear();
    }
    

    /***************************** Unimplemented *****************************/

    std::vector<IntVec2> HPA::find_bounded_path(const IntVec2& from_index, const IntVec2& to_index, const GridStamp& stamp,
        const IntVec2& bound_center, const int& bound_size) const 
    { return jps_finder->find_bounded_path(from_index,to_index,stamp,bound_center,bound_size);}

    std::optional<unsigned> HPA::get_path_length(const IntVec2& from_index, const IntVec2& to_index, const GridStamp& stamp) const 
    { return jps_finder->get_path_length(from_index, to_index, stamp); }

    std::optional<unsigned> HPA::get_bounded_path_length(const IntVec2& from_index, const IntVec2& to_index, const GridStamp& stamp,
        const IntVec2& bound_center, const int& bound_size) const 
    { return jps_finder->get_bounded_path_length(from_index, to_index, stamp, bound_center, bound_size);}

    bool HPA::check_unstuck(const IntVec2& center_index, const int& check_size, const GridStamp& stamp) const 
    { return jps_finder->check_unstuck(center_index,check_size,stamp); }


}