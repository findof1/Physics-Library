#ifndef RIGID_BODY_H
#define RIGID_BODY_H
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <map>
#include <memory>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#include "shader.h"

class RigidBody
{
public:
  glm::vec2 GRAVITY = glm::vec2(0.0f, -981.00f);
  glm::vec2 position;
  float rotation;
  float width;
  float height;

  float mass;
  glm::vec2 forceVector;
  glm::vec2 linearVelocity;

  float momentOfInertia;
  float torque;
  float angularVelocity;

  RigidBody(glm::vec2 position, float rotation, float width, float height, float mass);
  void update(double deltaTime);
  void applyForce(glm::vec2 force, glm::vec2 point = glm::vec2(0.0f, 0.0f));

  bool colliding(RigidBody rectangle);
};

#endif
