#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#include "Includes/shader.h"
#include "Includes/renderer.h"
#include "Includes/rigidBody.h"

void processInput(GLFWwindow *window);

bool darkMode = true;

int main()
{
	Renderer renderer("Physics Library");
	RigidBody square(glm::vec2(500.0f, 500.0f), 0.0f, 1.0f, 1.0f, 1.0f);

	double deltaTime;
	clock_t oldTime = clock();
	while (renderer.rendering())
	{
		clock_t currentTime = clock();
		deltaTime = static_cast<double>(currentTime - oldTime) / CLOCKS_PER_SEC;
		double fps = 1 / deltaTime;
		oldTime = currentTime;

		square.update(deltaTime);

		processInput(renderer.window);

		if (darkMode)
		{
			renderer.displayBackground(5, 5, 5, 1);
		}
		else
		{
			renderer.displayBackground(250, 250, 250, 1);
		}

		renderer.drawSquare(square.position, glm::vec2(100.0f, 100.0f), square.rotation, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));

		renderer.displayFrame();
	}

	renderer.close();
	return 0;
}

void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}