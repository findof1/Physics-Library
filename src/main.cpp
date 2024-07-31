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
	RigidBody square(glm::vec2(0.0f, 0.0f), 0.0f, 1.0f, 1.0f, 1.0f);

	double deltaTime;
	clock_t oldTime = clock();
	while (renderer.rendering())
	{
		deltaTime = clock() - oldTime;
		double fps = (1 / deltaTime) * 1000;
		oldTime = clock();

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

		renderer.drawSquare(glm::vec2(725.0f, 500.0f), glm::vec2(550.0f, 120.0f), 0.0f, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));

		renderer.drawCircle(glm::vec2(1225.0f, 700.0f), glm::vec2(500.0f, 500.0f), 0.0f, glm::vec4(0.0f, 0.5f, 0.5f, 1.0f));

		renderer.renderText("This is sample text", 500.0f, 500.0f, 1.0f, glm::vec3(1.0, 1.0f, 1.0f));

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