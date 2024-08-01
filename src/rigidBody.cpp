#include "Includes/rigidBody.h"

bool intervalsOverlap(float minA, float maxA, float minB, float maxB)
{
  return maxA >= minB && maxB >= minA;
}

float projectVertex(const glm::vec2 &vertex, const glm::vec2 &axis)
{
  return glm::dot(vertex, axis);
}

glm::vec2 computeNormal(const glm::vec2 &start, const glm::vec2 &end)
{
  return glm::vec2(end.y - start.y, start.x - end.x);
}

glm::vec2 getVertex(int index, RigidBody *rect)
{
  float halfWidth = rect->width / 2;
  float halfHeight = rect->height / 2;
  switch (index)
  {
  case 0:
    return glm::vec2(rect->position.x - halfWidth, rect->position.y + halfHeight);
  case 1:
    return glm::vec2(rect->position.x + halfWidth, rect->position.y + halfHeight);
  case 2:
    return glm::vec2(rect->position.x + halfWidth, rect->position.y - halfHeight);
  case 3:
    return glm::vec2(rect->position.x - halfWidth, rect->position.y - halfHeight);
  default:
    return glm::vec2(0.0f);
  }
}

glm::vec2 getNormal(int edgeIndex, RigidBody *rect)
{
  glm::vec2 start = getVertex(edgeIndex, rect);
  glm::vec2 end = getVertex((edgeIndex + 1) % 4, rect);
  return computeNormal(start, end);
}

RigidBody::RigidBody(glm::vec2 position, float rotation, float width, float height, float mass) : position(position), rotation(rotation), width(width), height(height), mass(mass)
{
  momentOfInertia = (1 / 12) * mass * (width * width + height * height);
}

void RigidBody::update(double deltaTime)
{
  applyForce(GRAVITY * mass, glm::vec2(position.x, position.y));

  glm::vec2 linearAcceleration = forceVector / mass;
  linearVelocity += glm::vec2(linearAcceleration.x * deltaTime, linearAcceleration.y * deltaTime);
  position += glm::vec2(linearVelocity.x * deltaTime, linearVelocity.y * deltaTime);

  float angularAcceleration = torque / mass;
  angularVelocity += angularAcceleration * deltaTime;
  rotation += angularVelocity * deltaTime;

  float angularFrictionCoefficient = 0.98f;
  angularVelocity *= angularFrictionCoefficient;

  forceVector = glm::vec2(0.0f, 0.0f);
}

bool RigidBody::colliding(RigidBody *rectangle)
{
  bool collision = true;

  for (int j = 0; j < 4; j++)
  {
    glm::vec2 axis = getNormal(j, this);

    float minA, maxA;
    minA = maxA = projectVertex(getVertex(0, this), axis);
    for (int i = 0; i < 4; i++)
    {

      float projection = projectVertex(getVertex(i, this), axis);
      minA = std::min(minA, projection);
      maxA = std::max(maxA, projection);
    }

    float minB, maxB;
    minB = maxB = projectVertex(getVertex(0, rectangle), axis);

    for (int i = 0; i < 4; i++)
    {

      float projection = projectVertex(getVertex(i, rectangle), axis);
      minB = std::min(minB, projection);
      maxB = std::max(maxB, projection);
    }

    if (!intervalsOverlap(minA, maxA, minB, maxB))
    {
      collision = false;
      break;
    }
  }

  for (int j = 0; j < 4; j++)
  {
    glm::vec2 axis = getNormal(j, rectangle);

    float minA, maxA;
    minA = maxA = projectVertex(getVertex(0, this), axis);
    for (int i = 0; i < 4; i++)
    {

      float projection = projectVertex(getVertex(i, this), axis);
      minA = std::min(minA, projection);
      maxA = std::max(maxA, projection);
    }

    float minB, maxB;
    minB = maxB = projectVertex(getVertex(0, rectangle), axis);

    for (int i = 0; i < 4; i++)
    {

      float projection = projectVertex(getVertex(i, rectangle), axis);
      minB = std::min(minB, projection);
      maxB = std::max(maxB, projection);
    }

    if (!intervalsOverlap(minA, maxA, minB, maxB))
    {
      collision = false;
      break;
    }
  }

  return collision;
}

void RigidBody::applyForce(glm::vec2 force, glm::vec2 point)
{
  forceVector += force;

  glm::vec2 offset = point - position;
  torque += glm::cross(glm::vec3(offset, 0.0f), glm::vec3(force, 0.0f)).z;
}
