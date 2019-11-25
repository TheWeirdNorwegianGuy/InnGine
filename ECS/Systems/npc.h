#ifndef NPC_H
#define NPC_H

#include "systembase.h"
#include "componentbase.h"
#include <vector>
#include "bspline.h"

class ECSManager;

class NPC : public SystemBase
{
public:
    NPC();

    gsl::Vec3 startPoint = orgStartPoint;
    gsl::Vec3 endPoint = orgEndPoint;
    std::vector<gsl::Vec3> controlPoints;


    enum NPCstates {PATROL, LEARN};
    enum NPCevents {ENDPOINT_ARRIVED, ITEM_TAKEN};

    NPCstates state;
    NPCevents event;

    float time;


    BSpline* spline = nullptr;

    static NPC getInstance();
    void patrol(GLfloat deltatime, size_t id, gsl::Vec3 pos);

    void learn(NPCevents &happening, NPCstates &status);

    void turnAround();

    void createSpline(std::vector<gsl::Vec3> &points, int degree);

    void addNode(gsl::Vec3 node);

    void removeNode(std::vector<gsl::Vec3> &points/*, trophy-entity*/);

    void npcUpdate(float t);


private:
    ECSManager* mManager{nullptr};
    static NPC* mInstance;

    gsl::Vec3 orgStartPoint{40.f, 70.f, 300.f};
    gsl::Vec3 orgEndPoint{100.f, 70.f, 380.f};
};

#endif // NPC_H
