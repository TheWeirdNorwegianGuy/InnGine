#include "npc.h"
#include "ecsmanager.h"
#include "vec2.h"
#include "vertex.h"
#include "meshbase.h"
#include "coreengine.h"
#include "gsl_math.h"

using namespace gsl;

NPC* NPC::mInstance = nullptr;


NPC::NPC()
{
    mInstance = this;
    mManager = ECSManager::getInstance();

    //create control points for bspline
    controlPoints.empty();

    addNode(startPoint);                            //Where the NPC starts
    for(int i = 0; i < mManager->mBoxColliderComponents.size(); i++)
    {
        //The position of every trophy
        addNode(mManager->mBoxColliderComponents[i].getFirstTransformComponent()->mTransform.getPosition());
    }
    addNode(endPoint);                              //Where it ends

    createSpline(controlPoints, 2);

    state = PATROL;
}

void NPC::patrol(GLfloat deltatime, size_t id, gsl::Vec3 pos)
{

    //Use patrol() on the ai

    std::vector<NPCComponent>& npc = mManager->mNPCComponents;
    TransformComponent* transform = npc[id].getFirstTransformComponent();

    //Gives speed and moves the AI
    transform->mTransform.setPosition(pos);

    Vec3 speed = transform->mTransform.getPosition() - npc[id].lastPosition;
    speed.setY(0.f);

    float speedMag = speed.length() / deltatime;

    if(speedMag > 12.f)
    {
        npc[id].speed -= 0.005f;
    }
    else if(speedMag < 12.f)
    {
        npc[id].speed += 0.005f;
    }

    speed.normalize();

    Vec3 rotAxis = speed^Vec3(0, 0, 1);

    double angle = gsl::rad2deg(static_cast<double>(std::acos(Vec3::dot(speed, Vec3(0, 0, -1)) / speed.length())));

    Quaternion rot(angle, rotAxis);
    transform->mTransform.setRotation(rot);

    npc[id].lastPosition = transform->mTransform.getPosition();
}

void NPC::learn(NPC::NPCevents &happening, NPC::NPCstates &status)
{
    switch(happening)
    {

    case ENDPOINT_ARRIVED:      //If npc has reach the end of the route, turn around and make a new route
    {
        turnAround();
        createSpline(controlPoints, 2);
        break;
    }/*
    case ITEM_TAKEN:            //If an item has been taken, remove its node from the vector
    {
        removeNode(trophyPoints, pickUp);
        break;
    }*/

    }

    status = PATROL;
}

void NPC::turnAround()
{
    gsl::Vec3 temp = startPoint;
    startPoint = endPoint;
    endPoint = temp;
}

void NPC::createSpline(std::vector<Vec3> &points, int degree)
{
    //Gather the trophie locations in the controlPoints vector
    controlPoints.empty();

    addNode(startPoint);                            //Where the NPC starts
    for(int i = 0; i < mManager->mBoxColliderComponents.size(); i++)
    {
        //The position of every trophy
        addNode(mManager->mBoxColliderComponents[i].getFirstTransformComponent()->mTransform.getPosition());
    }
    addNode(endPoint);

    if(!spline) //If there is no instance of bspline, make a new
    {
        spline = new BSpline(points, degree);
    }
    else        //If there is, use it
    {
    spline->makeSpline(points, degree);
    }
}

void NPC::addNode(Vec3 node)
{
    controlPoints.push_back(startPoint);

    for (size_t i = 0; i < mManager->mBoxColliderComponents.size(); i++)
    {
        if(mManager->mBoxColliderComponents[i].bSpline == true)
        {
            controlPoints.push_back(mManager->mBoxColliderComponents[i].getFirstTransformComponent()->mTransform.getPosition());
        }
    }
    controlPoints.push_back(endPoint);
}

void NPC::npcUpdate(GLfloat t)
{

    std::vector<NPCComponent>& npc = mManager->mNPCComponents;
    for (size_t i = 0; i < npc.size(); i++)
    {

        npc[i].path = spline;
        //If we have 1 or less controllPoints remaining the ai doesn't move
        if(controlPoints.size() >= 2)
        {
            Vec3 pos = spline->findPoint(npc[i].time);

            //Gets the transform of the AI to make it more accecible

            std::vector<MeshColliderComponent>& t3 = mManager->mMeshColliderComponents;
            Mat4 terrainMat, invTerrainMat;

            for(size_t k = 0; k < t3.size(); k++)
            {
                terrainMat = t3[k].modelMatrix;
                invTerrainMat = terrainMat;
                invTerrainMat.inverse();

                //Barycentric Coordinates
                int triangle = t3[k].mData->findTriangleIndexFromWorldPosition(npc[i].triangleLastFrame, pos, invTerrainMat, t3[k].triangles, t3[k].mData->vertices());
                if(triangle != -1)
                {
                    npc[i].triangleLastFrame = triangle;

                    float y = (npc[i].mData->mBoundingBoxRightUpFront).length();

                    Vec3 triangleCenter = (terrainMat*Vec4(npc[i].mData->centerFromTriangle(triangle, t3[k].triangles, t3[k].mData->vertices()), 1)).toVector3D();
                    Vec3 triangleNormal = (terrainMat*Vec4(npc[i].mData->normalFromTriangle(triangle, t3[k].triangles, t3[k].mData->vertices()), 0)).toVector3D().normalized();
                    Vec3 centerToTriangleCenter = pos - triangleCenter;

                    float distance = Vec3::dot(centerToTriangleCenter, triangleNormal);

                    pos.setY(pos.getY() - distance + y);
                }

                if(npc[i].time >= 0.999f)
                {
                    state = LEARN;
                    event = ENDPOINT_ARRIVED;
                    npc[i].time = 0;
                }
                if(state == LEARN)
                {
                    learn(event, state);
                }
                if(state == PATROL)
                {
                    patrol(t, i, pos);
                }

                npc[i].time += t*npc[i].speed;


            }
        }
        else
        {
            break;
        }
    }
}
