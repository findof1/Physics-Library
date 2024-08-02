#include "Includes/rigidBody.h"

bool intervalsOverlap(float minA, float maxA, float minB, float maxB)
{
  return maxA >= minB && maxB >= minA;
}

float projectVertex(const glm::vec2 &vertex, const glm::vec2 &axis)
{
  return glm::dot(vertex, axis);
}

glm::vec2 computeEdgeNormal(const glm::vec2 &start, const glm::vec2 &end)
{
  glm::vec2 edgeNormal = glm::vec2(end.y - start.y, start.x - end.x);
  if (glm::length(edgeNormal) == 0.0f)
  {
    return glm::vec2(0, 0);
  }
  return glm::normalize(edgeNormal);
}

glm::vec2 getVertex(int index, RigidBody *rect)
{
  float halfWidth = rect->width / 2;
  float halfHeight = rect->height / 2;

  glm::vec2 localVertices[4];
  localVertices[0] = glm::vec2(-halfWidth, halfHeight);
  localVertices[1] = glm::vec2(halfWidth, halfHeight);
  localVertices[2] = glm::vec2(halfWidth, -halfHeight);
  localVertices[3] = glm::vec2(-halfWidth, -halfHeight);

  glm::vec2 localVertex = localVertices[index];

  float angle = glm::radians(rect->rotation);
  glm::mat2 rotationMatrix = glm::mat2(
      glm::cos(angle), -glm::sin(angle),
      glm::sin(angle), glm::cos(angle));

  glm::vec2 rotatedVertex = rotationMatrix * localVertex;

  glm::vec2 worldVertex = rect->position + rotatedVertex;

  return worldVertex;
}

glm::vec2 getNormal(int edgeIndex, RigidBody *rect)
{
  glm::vec2 start = getVertex(edgeIndex, rect);
  glm::vec2 end = getVertex((edgeIndex + 1) % 4, rect);
  return computeEdgeNormal(start, end);
}

RigidBody::RigidBody(glm::vec2 position, float rotation, float width, float height, float mass) : position(position), rotation(rotation), width(width), height(height), mass(mass)
{
}

void RigidBody::update(double deltaTime)
{
  applyForce(GRAVITY * mass, glm::vec2(position.x, position.y));

  if (isStatic)
    return;

  glm::vec2 linearAcceleration = forceVector / mass;
  linearVelocity += glm::vec2(linearAcceleration.x * deltaTime, linearAcceleration.y * deltaTime);
  position += glm::vec2(linearVelocity.x * deltaTime, linearVelocity.y * deltaTime);

  float angularAcceleration = torque / mass;
  angularVelocity += angularAcceleration * deltaTime;
  rotation += angularVelocity * deltaTime;

  forceVector = glm::vec2(0.0f, 0.0f);
}

void RigidBody::resolveCollision(RigidBody *rectangle)
{
  if (isStatic && rectangle->isStatic)
    return;

  float minOverlap = FLT_MAX;
  glm::vec2 mtvAxis;
  glm::vec2 collisionPoint;

  for (int j = 0; j < 8; j++)
  {
    glm::vec2 axis;
    if (j < 4)
    {
      axis = getNormal(j, this);
    }
    else
    {
      axis = getNormal(j, rectangle);
    }

    if (axis == glm::vec2(0.0f, 0.0f))
      continue;

    float minA, maxA, minB, maxB;
    minA = maxA = projectVertex(getVertex(0, this), axis);
    for (int i = 1; i < 4; i++)
    {
      float projection = projectVertex(getVertex(i, this), axis);
      minA = std::min(minA, projection);
      maxA = std::max(maxA, projection);
    }

    minB = maxB = projectVertex(getVertex(0, rectangle), axis);
    for (int i = 1; i < 4; i++)
    {
      float projection = projectVertex(getVertex(i, rectangle), axis);
      minB = std::min(minB, projection);
      maxB = std::max(maxB, projection);
    }

    if (!intervalsOverlap(minA, maxA, minB, maxB))
    {
      return;
    }

    float overlapMin = std::max(minA, minB);
    float overlapMax = std::min(maxA, maxB);

    float overlap = std::max(0.0f, overlapMax - overlapMin);

    if (overlap < minOverlap)
    {
      minOverlap = overlap;
      mtvAxis = axis;
      float overlapCenter = (overlapMin + overlapMax) / 2.0f;

      collisionPoint = this->position + (overlapCenter - glm::dot(this->position, axis)) * axis;
    }
  }

  if (minOverlap > 0.0f)
  {
    glm::vec2 mtv = mtvAxis * minOverlap;

    if (glm::dot(mtv, position - rectangle->position) < 0)
    {
      mtv *= -2;
    }

    float momentOfInertia1 = (1.0f / 12.0f) * mass * (width * width + height * height);

    float momentOfInertia2 = (1.0f / 12.0f) * rectangle->mass * (rectangle->width * rectangle->width + rectangle->height * rectangle->height);

    glm::vec2 relativeVelocity = rectangle->linearVelocity - this->linearVelocity;
    float restitution = std::min(this->restitution, rectangle->restitution);

    float velocityAlongNormal = glm::dot(relativeVelocity, mtvAxis);

    float impulse = (-(1 + restitution) * velocityAlongNormal) / ((1 / this->mass) + (1 / rectangle->mass));

    glm::vec2 r1 = collisionPoint - this->position;
    glm::vec2 r2 = collisionPoint - rectangle->position;

    float angularImpulse1 = glm::dot(glm::cross(glm::vec3(r1, 0.0f), glm::vec3(mtvAxis, 0.0f)), glm::vec3(0.0f, 0.0f, 1.0f)) * impulse;
    float angularImpulse2 = glm::dot(glm::cross(glm::vec3(r2, 0.0f), glm::vec3(mtvAxis, 0.0f)), glm::vec3(0.0f, 0.0f, 1.0f)) * impulse;

    if ((abs(collisionPoint.x - this->position.x) < 0.1 || abs(collisionPoint.y - this->position.y) < 0.1 || abs(collisionPoint.x - rectangle->position.x) < 0.1 || abs(collisionPoint.y - rectangle->position.y) < 0.1) || this->rotation == 0 || rectangle->rotation == 0)
    {
      this->angularVelocity += angularImpulse1 / momentOfInertia1;
      rectangle->angularVelocity -= angularImpulse2 / momentOfInertia2;
    }

    glm::vec2 impulseVector = impulse * mtvAxis;

    this->linearVelocity -= impulseVector / this->mass;
    rectangle->linearVelocity += impulseVector / rectangle->mass;

    if (this->isStatic)
    {
      rectangle->position -= mtv;
      return;
    }

    this->position += mtv;
  }
}

void RigidBody::applyForce(glm::vec2 force, glm::vec2 point)
{
  forceVector += force;

  glm::vec2 offset = point - position;
  torque += glm::cross(glm::vec3(offset, 0.0f), glm::vec3(force, 0.0f)).z;
}

void RigidBody::applyTorque(float torqueAdd)
{
  torque += torqueAdd;
}
