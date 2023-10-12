#ifndef MESHCOMPONENT_H
#define MESHCOMPONENT_H

#include <la.h>
#include <QListWidgetItem>

class Vertex;
class HalfEdge;
class Face;

class Vertex : public QListWidgetItem
{
private:
    inline static unsigned int GID = 0;
    unsigned int ID;

public:
    glm::vec3 pos;
    HalfEdge* he;

    Vertex(const glm::vec3& p = glm::vec3(0), HalfEdge* h = nullptr)
        : QListWidgetItem(), pos(p), he(h), ID(GID++) {}

    unsigned int getID() const {return ID;}
    static void resetGID() {GID = 0;}
};

class Face : public QListWidgetItem
{
private:
    inline static unsigned int GID = 0;
    unsigned int ID;

public:
    glm::vec3 color;
    HalfEdge* he;


    Face(const glm::vec3& c = glm::vec3(0), HalfEdge* h = nullptr)
        : QListWidgetItem(), color(c), he(h), ID(GID++) {}

    unsigned int getID() const {return ID;}
    static void resetGID() {GID = 0;}
};

class HalfEdge : public QListWidgetItem
{
private:
    inline static unsigned int GID = 0;
    unsigned int ID;

public:
    HalfEdge* next;
    HalfEdge* sym;
    Face* face;
    Vertex* vert;

    HalfEdge() : QListWidgetItem(), next(nullptr), sym(nullptr),
        face(nullptr), vert(nullptr), ID(GID++) {}

    unsigned int getID() const {return ID;}
    static void resetGID() {GID = 0;}

};

#endif // MESHCOMPONENT_H
