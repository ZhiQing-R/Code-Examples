#ifndef MESH_H
#define MESH_H

#include "drawable.h"
#include "smartpointerhelp.h"
#include "meshcomponent.h"
#include "ray.hpp"

class Mesh : public Drawable
{
typedef glm::highp_u32vec3 uvec3;
private:
    std::vector<uPtr<Vertex>> vertices;
    std::vector<uPtr<Face>> faces;
    std::vector<uPtr<HalfEdge>> halfedges;

public:
    Mesh(OpenGLContext *mp_context);
    virtual void create() override;
    virtual GLenum drawMode() override;
    void loadOBJ(const char* filename);
    void generateHalfedge(const std::vector<glm::vec3>& v,
                          const std::vector<std::vector<uvec3>>& f);
    void setListItems(QListWidget* vparent, QListWidget* fparent,
                        QListWidget* hparent);

    Vertex* getIntersectVertex(const Ray& ray) const;
    Face* getIntersectFace(const Ray& ray) const;
    HalfEdge* getIntersectHalfEdge(const Ray& ray) const;

};

#endif // MESH_H
