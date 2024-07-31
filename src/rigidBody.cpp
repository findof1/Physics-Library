#include "Includes/rigidBody.h"

RigidBody::RigidBody(glm::vec2 position, float rotation, float width, float height, float mass) : position(position), rotation(rotation), width(width), height(height), mass(mass)
{
  momentOfInertia = (1 / 12) * mass * (width * width + height * height);
}

void RigidBody::update(double deltaTime)
{
  applyForce(glm::vec2(0.0f, -1.0f));

  glm::vec2 acceleration = forceVector / mass;
  linearVelocity += glm::vec2(acceleration.x * deltaTime, acceleration.y * deltaTime);

  position += glm::vec2(linearVelocity.x * deltaTime, linearVelocity.y * deltaTime);

  forceVector = glm::vec2(0.0f, 0.0f);

  std::cout << position.x << position.y << std::endl;
}

void RigidBody::applyForce(glm::vec2 force)
{
  forceVector += force;
}