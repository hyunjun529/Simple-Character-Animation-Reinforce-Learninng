#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "Utils/b3Clock.h"

#include "OpenGLWindow/SimpleOpenGL3App.h"
#include <stdio.h>
#include "ExampleBrowser/OpenGLGuiHelper.h"

CommonExampleInterface* example;
int gSharedMemoryKey = -1;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove(float x, float y)
{
	bool handled = false;
	handled = example->mouseMoveCallback(x, y);
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback(x, y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback = 0;
static void OnMouseDown(int button, int state, float x, float y) {
	bool handled = false;

	handled = example->mouseButtonCallback(button, state, x, y);
	if (!handled)
	{
		if (prevMouseButtonCallback)
			prevMouseButtonCallback(button, state, x, y);
	}
}

/* keyboard */
b3KeyboardCallback prevKeyboardCallback = 0;
static void OnKeyboard(int key, int state) {
	bool handled = false;

	handled = example->keyboardCallback(key, state);
	if (!handled)
	{
		if (prevKeyboardCallback)
			prevKeyboardCallback(key, state);
	}
}

class LessDummyGuiHelper : public DummyGUIHelper
{
	CommonGraphicsApp* m_app;
public:
	virtual CommonGraphicsApp* getAppInterface()
	{
		return m_app;
	}

	LessDummyGuiHelper(CommonGraphicsApp* app)
		:m_app(app)
	{
	}
};
int main(int argc, char* argv[])
{

	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Standalone Example", 800, 600, true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

	/* keyboard */
	app->m_window->setKeyboardCallback((b3KeyboardCallback)OnKeyboard);

	OpenGLGuiHelper gui(app, false);
	//LessDummyGuiHelper gui(app);
	//DummyGUIHelper gui;

	CommonExampleOptions options(&gui);

	example = StandaloneExampleCreateFunc(options);
	example->initPhysics();
	example->resetCamera();

	b3Clock clock;

	do
	{
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		btScalar dtSec = btScalar(clock.getTimeInSeconds());
		example->stepSimulation(dtSec);
		clock.reset();

		example->renderScene();

		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);

		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	example->exitPhysics();
	delete example;
	delete app;
	return 0;
}

