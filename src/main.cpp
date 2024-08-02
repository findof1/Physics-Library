#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#include "Includes/shader.h"
#include "Includes/renderer.h"
#include "Includes/rigidBody.h"

void processInput(GLFWwindow *window);

bool darkMode = true;

Renderer renderer("Physics Library");
RigidBody square(glm::vec2(500.0f, 500.0f), 0.0f, 100.0f, 100.0f, 1.0f);

RigidBody square2(glm::vec2(700.0f, 500.0f), 0.0f, 100.0f, 100.0f, 1.0f);

RigidBody square3(glm::vec2(400.0f, 200.0f), 0.0f, 1000.0f, 100.0f, 1.0f);

int main()
{
	// square.GRAVITY = glm::vec2(0.0f, 0.0f);
	// square2.GRAVITY = glm::vec2(0.0f, 0.0f);
	// square3.GRAVITY = glm::vec2(0.0f, 0.0f);
	square3.isStatic = true;

	float deltaTime;
	clock_t oldTime = clock();
	while (renderer.rendering())
	{
		clock_t currentTime = clock();
		deltaTime = static_cast<double>(currentTime - oldTime) / CLOCKS_PER_SEC;
		float fps = 1 / deltaTime;
		if (abs(deltaTime) < 0.00001)
		{
			fps = 500;
		}
		oldTime = currentTime;

		square.update(deltaTime);
		square2.update(deltaTime);
		square3.update(deltaTime);

		processInput(renderer.window);

		if (darkMode)
		{
			renderer.displayBackground(5, 5, 5, 1);
		}
		else
		{
			renderer.displayBackground(250, 250, 250, 1);
		}

		renderer.drawSquare(square.position, glm::vec2(square.width, square.height), square.rotation, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));

		renderer.drawSquare(square2.position, glm::vec2(square2.width, square2.height), square2.rotation, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));

		renderer.drawSquare(square3.position, glm::vec2(square3.width, square3.height), square3.rotation, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));

		square.resolveCollision(&square2);
		square.resolveCollision(&square3);
		square2.resolveCollision(&square3);

		renderer.renderText("FPS: " + std::to_string(fps), 1000, 1000, 1, glm::vec3(1.0f));

		renderer.displayFrame();
	}

	renderer.close();
	return 0;
}

void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		square.applyForce(glm::vec2(50, 0), glm::vec2(square.position.x, square.position.y));
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		square.applyForce(glm::vec2(-50, 0), glm::vec2(square.position.x, square.position.y));
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		square.applyForce(glm::vec2(0, 50), glm::vec2(square.position.x, square.position.y));
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		square.applyForce(glm::vec2(0, -50), glm::vec2(square.position.x, square.position.y));
	if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS)
		square.applyForce(glm::vec2(1, 0), glm::vec2(square.position.x, square.position.y - 1));
	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
		square.applyForce(glm::vec2(1, 0), glm::vec2(square.position.x, square.position.y + 1));
}