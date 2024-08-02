#ifndef RENDERER_H
#define RENDERER_H
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <map>
#include <memory>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#include "shader.h"

struct Character
{
  unsigned int TextureID;
  glm::ivec2 Size;
  glm::ivec2 Bearing;
  unsigned int Advance;
};

class Renderer
{
public:
  GLFWwindow *window;
  int ScreenW = 1920;
  int ScreenH = 1080;

  float zoom = 1;

  Renderer(std::string windowName);
  bool rendering();

  void displayFrame();
  void displayBackground(float r, float g, float b, float a);
  void close();

  void drawSquare(glm::vec2 position, glm::vec2 scale, float rotation, glm::vec4 color);
  void drawCircle(glm::vec2 position, glm::vec2 scale, float rotation, glm::vec4 color);
  void drawVector(glm::vec2 startPosition, glm::vec2 vector, glm::vec4 color);

  void renderText(std::string text, float x, float y, float scale, glm::vec3 color);

private:
  GLuint SquareVAO, SquareVBO, SquareEBO;
  GLuint CircleVBO, CircleVAO;
  std::map<GLchar, Character> Characters;
  GLuint TextVAO, TextVBO;

  std::unique_ptr<Shader> shader;
  std::unique_ptr<Shader> textShader;

  void initGLFW(std::string windowName);
  void initGlad();
  void initFreeType2();
  void initSquareBuffers();
  void initCircleBuffers();
};

#endif
