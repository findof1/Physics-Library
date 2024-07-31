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
  glm::vec2 position;
  float rotation;
  float width;
  float height;

  float mass;
  float momentOfInertia;
  glm::vec2 linearVelocity;
  float angularVelocity;
  glm::vec2 forceVector;

  

  RigidBody(glm::vec2 position, float rotation, float width, float height, float mass);
  void update(double deltaTime);
  void applyForce(glm::vec2 force);
};

#endif
