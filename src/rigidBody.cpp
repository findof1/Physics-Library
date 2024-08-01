#include "Includes/rigidBody.h"

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

bool RigidBody::colliding(RigidBody rectangle)
{
  glm::vec2 l1 = glm::vec2(position.x - width / 2, position.y + height / 2);
  glm::vec2 r1 = glm::vec2(position.x + width / 2, position.y - height / 2);
  glm::vec2 l2 = glm::vec2(rectangle.position.x - rectangle.width / 2, rectangle.position.y + rectangle.height / 2);
  glm::vec2 r2 = glm::vec2(rectangle.position.x + rectangle.width / 2, rectangle.position.y - rectangle.height / 2);

  if (l1.x == r1.x || l1.y == r1.y || r2.x == l2.x || l2.y == r2.y)
    return false;

  if (l1.x > r2.x || l2.x > r1.x)
    return false;

  if (r1.y > l2.y || r2.y > l1.y)
    return false;

  return true;
}

void RigidBody::applyForce(glm::vec2 force, glm::vec2 point)
{
  forceVector += force;

  glm::vec2 offset = point - position;
  torque += glm::cross(glm::vec3(offset, 0.0f), glm::vec3(force, 0.0f)).z;
}