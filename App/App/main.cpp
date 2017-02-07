#include <stdio.h>

#include "Utils/b3Clock.h"

#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"

#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"

#include "ExampleImporter.h"

CommonExampleInterface* example;

b3MouseMoveCallback prevMouseMoveCallback = 0;
b3MouseButtonCallback prevMouseButtonCallback = 0;
b3KeyboardCallback prevKeyboardCallback = 0;

static void OnMouseMove(float x, float y);
static void OnMouseDown(int button, int state, float x, float y);
static void OnKeyboard(int key, int state);

static void SystemAlert(const char* msg);
bool switchRendering = true;

int main(int argc, char* argv[]) {

	// select Example
	b3Printf("%s\n", getExamples["000"].m_name);

	SimpleOpenGL3App* app = new SimpleOpenGL3App("000_HelloWorld", 800, 600, true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();
	
	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);
	app->m_window->setKeyboardCallback((b3KeyboardCallback)OnKeyboard);
	
	OpenGLGuiHelper gui(app, true);
	CommonExampleOptions options(&gui);

	example = getExamples["000"].m_createFunc(options);
	example->initPhysics();
	example->resetCamera();

	b3Clock clock;

	do
	{
		if (switchRendering) {
			app->m_instancingRenderer->init();
			app->m_instancingRenderer->updateCamera(app->getUpAxis());
		}

		btScalar dtSec = btScalar(clock.getTimeInSeconds());
		example->stepSimulation(dtSec);
		clock.reset();

		if (switchRendering) {
			example->renderScene();

			DrawGridData dg;
			dg.upAxis = app->getUpAxis();
			app->drawGrid(dg);
		}

		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	example->exitPhysics();
	delete example;
	delete app;
	return 0;
}

static void OnMouseMove(float x, float y) {
	bool handled = false;
	handled = example->mouseMoveCallback(x, y);
	if (!handled) {
		if (prevMouseMoveCallback)
			prevMouseMoveCallback(x, y);
	}
}

static void OnMouseDown(int button, int state, float x, float y) {
	bool handled = false;

	handled = example->mouseButtonCallback(button, state, x, y);
	if (!handled) {
		if (prevMouseButtonCallback)
			prevMouseButtonCallback(button, state, x, y);
	}
}

static void OnKeyboard(int key, int state) {
	bool handled = false;

	handled = example->keyboardCallback(key, state);

	if (state) {
		switch (key) {
		case B3G_ESCAPE:
			switchRendering = switchRendering ? false : true;
			if (switchRendering) SystemAlert("Graphic On");
			else SystemAlert("Graphic Off");
			break;
		}
	}

	if (!handled) {
		if (prevKeyboardCallback)
			prevKeyboardCallback(key, state);
	}
}

static void SystemAlert(const char* msg) {
	b3Printf("\n");
	b3Printf("============================================================\n");
	b3Printf("     SYSTEM ALERT : %s\n", msg);
	b3Printf("============================================================\n");
	b3Printf("\n");
}