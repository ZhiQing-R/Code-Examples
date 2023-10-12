#pragma once

#include <mesh.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

typedef unsigned int uint;
typedef std::pair<uint, uint> vpair;

Mesh::Mesh(OpenGLContext *mp_context)
    : Drawable(mp_context),
    vertices(), faces(), halfedges() {}

static std::vector<std::string> stringSplit(const std::string& s, char sep)
{
    std::vector<std::string> rtn;
    size_t last = 0, next = 0;
    while ((next = s.find(sep, last)) != std::string::npos)
    {
        rtn.push_back(s.substr(last, next - last));
        last = next + 1;
    }
    rtn.push_back(s.substr(last));
    return rtn;
}


void Mesh::loadOBJ(const char* filename)
{
    typedef glm::highp_u32vec3 uvec3;
    std::ifstream ifs(filename);

    std::vector<glm::vec3> v;
    std::vector<glm::vec3> vn;
    std::vector<glm::vec2> vt;
    std::vector<std::vector<uvec3>> face;

    // allocate char buffer
    int maxchars = 8192;
    std::vector<char> buf(maxchars);

    while (ifs.peek() != -1)
    {
        ifs.getline(&buf[0], maxchars);
        std::string linebuf(&buf[0]);

        if (linebuf.size() > 0)
        {
            if (linebuf[linebuf.size() - 1] == '\n')
                linebuf.erase(linebuf.size() - 1);
        }
        if (linebuf.size() > 0)
        {
            if (linebuf[linebuf.size() - 1] == '\r')
                linebuf.erase(linebuf.size() - 1);
        }

        if (linebuf.empty())
            continue;

        if (linebuf[0] == '\0')
            continue; // empty line

        if (linebuf[0] == '#')
            continue; // comment line

        if (linebuf[0] == 'v' && linebuf[1] == ' ')
        {
            std::vector<std::string> elements = stringSplit(linebuf, ' ');
            glm::vec3 tmp = glm::vec3(std::stof(elements[1]), std::stof(elements[2]), std::stof(elements[3]));
            v.push_back(tmp);
            continue;
        }

        if (linebuf[0] == 'v' && linebuf[1] == 'n' && linebuf[2] == ' ')
        {
            std::vector<std::string> elements = stringSplit(linebuf, ' ');
            glm::vec3 tmp = glm::vec3(std::stof(elements[1]), std::stof(elements[2]), std::stof(elements[3]));
            vn.push_back(tmp);
            continue;
        }

        if (linebuf[0] == 'v' && linebuf[1] == 't' && linebuf[2] == ' ')
        {
            std::vector<std::string> elements = stringSplit(linebuf, ' ');
            glm::vec2 tmp = glm::vec2(std::stof(elements[1]), std::stof(elements[2]));
            vt.push_back(tmp);
            continue;
        }

        if (linebuf[0] == 'f' && linebuf[1] == ' ')
        {
            std::vector<std::string> elements = stringSplit(linebuf, ' ');
            std::vector<uvec3> fe;
            for (size_t i = 1; i < elements.size(); ++i)
            {
                std::vector<std::string> vvnvt = stringSplit(elements[i], '/');
                fe.push_back(uvec3(std::stoi(vvnvt[0]), std::stoi(vvnvt[1]), std::stoi(vvnvt[2])));
            }
            face.push_back(fe);
            continue;
        }

    }

    generateHalfedge(v, face);
}

struct vpairHash
{
    std::size_t operator () (const vpair& p) const
    {
        auto h1 = std::hash<uint>{}(p.first);
        auto h2 = std::hash<uint>{}(p.second);
        return h1 ^ h2;
    }
};

void Mesh::generateHalfedge(const std::vector<glm::vec3>& v,
                            const std::vector<std::vector<uvec3>>& f)
{
    vertices.clear();
    faces.clear();
    halfedges.clear();
    Vertex::resetGID();
    Face::resetGID();
    HalfEdge::resetGID();

    std::unordered_map<vpair, HalfEdge*, vpairHash> heTable;

    // initialize all vertices
    for (const auto& pos : v)
    {
        vertices.emplace_back(mkU<Vertex>(pos));
    }

    for (const auto& face : f)
    {
        // ignore non-closed face
        if (face.size() < 3) continue;

        Face* firstF = faces.emplace_back(mkU<Face>()).get();
        firstF->color = glm::vec3((float) rand() / RAND_MAX, (float) rand() / RAND_MAX,
                                  (float) rand() / RAND_MAX);
        Vertex* currentV;
        Vertex* lastV;
        HalfEdge* currentH;
        HalfEdge* lastH;
        bool firstEdge = true;

        for (const auto& fg : face)
        {
            size_t vidx = fg[0] - 1;
            currentV = vertices[vidx].get();
            currentH = halfedges.emplace_back(mkU<HalfEdge>()).get();

            currentH->vert = currentV;
            currentV->he = currentH;

            currentH->face = firstF;

            if (firstEdge)
            {
                firstF->he = currentH;
                lastH = currentH;
                lastV = currentV;
                firstEdge = false;
                continue;
            }

            lastH->next = currentH;

            // sym check
            vpair key = vpair(currentV->getID(), lastV->getID());
            if (heTable.find(key) != heTable.end())
            {
                heTable[key]->sym = currentH;
                currentH->sym = heTable[key];
            }
            else
            {
                key = vpair(lastV->getID(), currentV->getID());
                heTable[key] = currentH;
            }
            lastH = currentH;
            lastV = currentV;
        }
        lastH->next = firstF->he;
        // sym check for the first halfedge
        vpair key = vpair(firstF->he->vert->getID(), lastV->getID());
        if (heTable.find(key) != heTable.end())
        {
            heTable[key]->sym = firstF->he;
            firstF->he->sym = heTable[key];
        }
        else
        {
            key = vpair(lastV->getID(), firstF->he->vert->getID());
            heTable[key] = firstF->he;
        }

    }

}

void Mesh::create()
{
    std::vector<glm::vec4> pos;
    std::vector<glm::vec4> nor;
    std::vector<glm::vec4> col;
    std::vector<GLuint> idx;
    count = 0;

    for (const auto& f : faces)
    {
        int vertCount = 0;
        HalfEdge* curr = f->he;

        Vertex* v1 = curr->vert;
        Vertex* v2 = curr->next->vert;
        Vertex* v3 = curr->next->next->vert;
        glm::vec3 normal = glm::normalize(glm::cross(v2->pos - v1->pos, v3->pos - v2->pos));
        do
        {
            ++vertCount;
            pos.push_back(glm::vec4(curr->vert->pos, 1));
            nor.push_back(glm::vec4(normal, 0));
            col.push_back(glm::vec4(f->color, 1));
            curr = curr->next;

        } while(curr != f->he);

        for (int i = 0; i < vertCount - 2; ++i)
        {
            idx.push_back(count);
            idx.push_back(count + i + 1);
            idx.push_back(count + i + 2);
        }

        count += vertCount;
    }
    count = idx.size();

    generateIdx();
    mp_context->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufIdx);
    mp_context->glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size() * sizeof(GLuint), idx.data(), GL_STATIC_DRAW);

    generatePos();
    mp_context->glBindBuffer(GL_ARRAY_BUFFER, bufPos);
    mp_context->glBufferData(GL_ARRAY_BUFFER, pos.size() * sizeof(glm::vec4), pos.data(), GL_STATIC_DRAW);

    generateNor();
    mp_context->glBindBuffer(GL_ARRAY_BUFFER, bufNor);
    mp_context->glBufferData(GL_ARRAY_BUFFER, nor.size() * sizeof(glm::vec4), nor.data(), GL_STATIC_DRAW);

    generateCol();
    mp_context->glBindBuffer(GL_ARRAY_BUFFER, bufCol);
    mp_context->glBufferData(GL_ARRAY_BUFFER, col.size() * sizeof(glm::vec4), col.data(), GL_STATIC_DRAW);

}

GLenum Mesh::drawMode()
{
    return GL_TRIANGLES;
}

void Mesh::setListItems(QListWidget* vparent, QListWidget* fparent,
                        QListWidget* hparent)
{
    for (const auto& v : vertices)
    {
        v->setText(QString::fromStdString(std::to_string(v->getID())));
        vparent->addItem(v.get());
    }

    for (const auto& f : faces)
    {
        f->setText(QString::fromStdString(std::to_string(f->getID())));
        fparent->addItem(f.get());
    }

    for (const auto& h : halfedges)
    {
        h->setText(QString::fromStdString(std::to_string(h->getID())));
        hparent->addItem(h.get());
    }
}

Vertex* Mesh::getIntersectVertex(const Ray& ray) const
{
    Vertex* rst = nullptr;
    float minT = FLT_MAX;

    for (const auto& v : vertices)
    {
        float t = raySphereIntersect(ray, v->pos);
        if (t < minT)
        {
            minT = t;
            rst = v.get();
        }
    }
    return rst;
}

Face* Mesh::getIntersectFace(const Ray& ray) const
{
    Face* rst = nullptr;
    float minT = FLT_MAX;

    for (const auto& face : faces)
    {
        std::vector<glm::vec3> pos;
        HalfEdge* curr = face->he;
        do
        {
            pos.push_back(curr->vert->pos);
            curr = curr->next;

        } while(curr != face->he);

        for (int i = 0; i < pos.size() - 2; ++i)
        {
            float t = rayTriangleIntersection(ray, pos[0], pos[i + 1], pos[i + 2]);
            if (t < minT)
            {
                minT = t;
                rst = face.get();
            }
        }

    }

    return rst;
}

HalfEdge* Mesh::getIntersectHalfEdge(const Ray& ray) const
{
    HalfEdge* rst = nullptr;
    float minT = FLT_MAX;

    for (const auto& halfedge : halfedges)
    {
        HalfEdge* he = halfedge.get();
        HalfEdge* curr = he;
        while (curr->next != he)
        {
            curr = curr->next;
        }

        float t = rayCylinderIntersect(ray, curr->vert->pos, he->vert->pos);
        if (t < minT)
        {
            minT = t;
            rst = he;
        }
    }
    return rst;
}
