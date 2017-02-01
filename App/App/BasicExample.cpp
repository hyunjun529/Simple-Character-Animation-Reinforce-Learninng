#include "BasicExample.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "CommonInterfaces/CommonRigidBodyBase.h"
#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#include "LinearHypothesis.h"
#include "ActionRecorder.h"

LinearHypothesis lh;
ActionRecorder AR;
bool needStuding = true;

int num = 0;
short collisionFilterGroup = short(btBroadphaseProxy::CharacterFilter);
short collisionFilterMask = short(btBroadphaseProxy::AllFilter ^ (btBroadphaseProxy::CharacterFilter));
static btScalar radius(0.2);

struct BasicExample : public CommonRigidBodyBase
{
	bool m_once;
	btAlignedObjectArray<btJointFeedback*> m_jointFeedback;
	btHingeConstraint* hinge;
	btRigidBody* linkBody;
	btVector3 groundOrigin_target;
	btRigidBody* body;
	BasicExample(struct GUIHelperInterface* helper);
	virtual ~BasicExample();
	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);
	void lockLiftHinge(void);

	virtual bool keyboardCallback(int key, int state);
	virtual void resetCamera()
	{

		float dist = 5;
		float pitch = 270;
		float yaw = 21;
		float targetPos[3] = { -1.34,3.4,-0.44 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
};

BasicExample::BasicExample(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper),
	m_once(true)
{
}

BasicExample::~BasicExample()
{
	for (int i = 0; i<m_jointFeedback.size(); i++)
	{
		delete m_jointFeedback[i];
	}

}

/********************************************************************************************
* start custom Functions
*********************************************************************************************/

void initState(BasicExample* target) {
	target->m_guiHelper->removeAllGraphicsInstances();
	target->initPhysics();
}

void moveLeft(btHingeConstraint *target) {
	target->setLimit(-M_PI / 1.0f, M_PI / 1.0f);
	target->enableAngularMotor(true, -15.0, 4000.f);
	
}

void moveRight(btHingeConstraint *target) {
	target->setLimit(-M_PI / 1.0f, M_PI / 1.0f);
	target->enableAngularMotor(true, 15.0, 4000.f);
}

/********************************************************************************************
* end custom Functions
*********************************************************************************************/

void BasicExample::stepSimulation(float deltaTime)
{
	if (0)//m_once)
	{
		m_once = false;
		btHingeConstraint* hinge = (btHingeConstraint*)m_dynamicsWorld->getConstraint(0);

		btRigidBody& bodyA = hinge->getRigidBodyA();
		btTransform trA = bodyA.getWorldTransform();
		btVector3 hingeAxisInWorld = trA.getBasis()*hinge->getFrameOffsetA().getBasis().getColumn(2);
		hinge->getRigidBodyA().applyTorque(-hingeAxisInWorld * 10);
		hinge->getRigidBodyB().applyTorque(hingeAxisInWorld * 10);

	}

	//get distance
	btScalar distance = sqrt(pow((groundOrigin_target.getZ() - linkBody->getCenterOfMassPosition().getZ()), 2) + pow((groundOrigin_target.getY() - linkBody->getCenterOfMassPosition().getY()), 2)) - 0.225;
	//b3Printf("distance = %f\n", distance);

	//collison check
	int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++)
	{
		if (m_dynamicsWorld->getDispatcher()->getNumManifolds() == 0) continue;
		btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();

		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < 0.f)
			{
				const btVector3& ptA = pt.getPositionWorldOnA();
				const btVector3& ptB = pt.getPositionWorldOnB();
				const btVector3& normalOnB = pt.m_normalWorldOnB;
				initState(this);
				b3Printf("check\n");

				/********************************************************************************************
				* start RL
				*********************************************************************************************/

				float lr = 0.5f;

				// LR
				for(int t = 0; t < 10000; t++) {
					for (int i = 0; i < AR.memory.num_elements; i++) {
						float a = (float)AR.memory.moved_array[i];
						float b = AR.memory.reward_array[i]; 

						const float error = b - lh.getY(a);
						const float sqr_error = 0.5 * error * error;

						const float da = 2.0 * error * -a;
						const float db = 2.0 * error * -1;

						lh.a -= da * lr;
						lh.b -= db * lr;
					}
				}
				
				needStuding = false;

				AR.clearHistory();

				/********************************************************************************************
				* end RL
				*********************************************************************************************/
			}
		}
	}

	/********************************************************************************************
	* start select Move
	*********************************************************************************************/

	
	int thisMoved = -1;

	
	if(needStuding){
		// random move
		thisMoved = ((int)rand() % 2 == 0) ? (1):(0);
	}
	else {
		// learned move
		thisMoved = (lh.getY(distance) >= 0.5)?(0):(1);
	}

	// moving
	if (thisMoved == ACT_MOVE_LEFT) {
		moveLeft(hinge);
	}
	else {
		moveRight(hinge);
	}

	// check current
	b3Printf("!!?!moved = %d\tdistance=%f\ttcurrent Y = %f\n", thisMoved, distance, lh.getY(distance));

	

	// Memory
	AR.recordHistory(thisMoved, distance);

	/********************************************************************************************
	* end select RL
	*********************************************************************************************/

	m_dynamicsWorld->stepSimulation(1. / 240, 0);

	static int count = 0;
}

void BasicExample::initPhysics()
{
	int upAxis = 1;
	m_guiHelper->setUpAxis(upAxis);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = false;

	m_dynamicsWorld->setGravity(btVector3(0, 0, -10));

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	int mode = btIDebugDraw::DBG_DrawWireframe
		+ btIDebugDraw::DBG_DrawConstraints
		+ btIDebugDraw::DBG_DrawConstraintLimits;
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(mode);


	{ // create a door using hinge constraint attached to the world

		int numLinks = 2;
		bool spherical = false;					//set it ot false -to use 1DoF hinges instead of 3DoF sphericals
		bool canSleep = false;
		bool selfCollide = false;

		btVector3 baseHalfExtents(0.05, 0.37, 0.1);
		btVector3 linkHalfExtents(0.05, 0.37, 0.1);

		btBoxShape* baseBox = new btBoxShape(baseHalfExtents);
		btVector3 basePosition = btVector3(-0.4f, 4.f, 0.f);
		btTransform baseWorldTrans;
		baseWorldTrans.setIdentity();
		baseWorldTrans.setOrigin(basePosition);

		//mbC->forceMultiDof();							//if !spherical, you can comment this line to check the 1DoF algorithm
		//init the base
		btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
		float baseMass = 0.f;
		float linkMass = 1.f;

		btRigidBody* base = createRigidBody(baseMass, baseWorldTrans, baseBox);
		m_dynamicsWorld->removeRigidBody(base);
		base->setDamping(0, 0);
		m_dynamicsWorld->addRigidBody(base, collisionFilterGroup, collisionFilterMask);
		btBoxShape* linkBox1 = new btBoxShape(linkHalfExtents);
		btSphereShape* linkSphere = new btSphereShape(radius);

		btRigidBody* prevBody = base;

		for (int i = 0; i<numLinks; i++)
		{
			btTransform linkTrans;
			linkTrans = baseWorldTrans;

			linkTrans.setOrigin(basePosition - btVector3(0, linkHalfExtents[1] * 2.f*(i + 1), 0));

			btCollisionShape* colOb = 0;

			if (i == 0)
			{
				colOb = linkBox1;
			}
			else
			{
				colOb = linkSphere;
			}
			linkBody = createRigidBody(linkMass, linkTrans, colOb);
			m_dynamicsWorld->removeRigidBody(linkBody);
			m_dynamicsWorld->addRigidBody(linkBody, collisionFilterGroup, collisionFilterMask);
			linkBody->setDamping(0, 0);
			btTypedConstraint* con = 0;

			if (i == 0)
			{
				//create a hinge constraint
				btVector3 pivotInA(0, -linkHalfExtents[1], 0);
				btVector3 pivotInB(0, linkHalfExtents[1], 0);
				btVector3 axisInA(1, 0, 0);
				btVector3 axisInB(1, 0, 0);
				bool useReferenceA = true;
				hinge = new btHingeConstraint(*prevBody, *linkBody,
					pivotInA, pivotInB,
					axisInA, axisInB, useReferenceA);
				hinge->setLimit(0.0f, 0.0f);
				m_dynamicsWorld->addConstraint(hinge, true);
				con = hinge;
			}
			else
			{
				btTransform pivotInA(btQuaternion::getIdentity(), btVector3(0, -radius, 0));						//par body's COM to cur body's COM offset
				btTransform pivotInB(btQuaternion::getIdentity(), btVector3(0, radius, 0));							//cur body's COM to cur body's PIV offset
				btGeneric6DofSpring2Constraint* fixed = new btGeneric6DofSpring2Constraint(*prevBody, *linkBody,
					pivotInA, pivotInB);
				fixed->setLinearLowerLimit(btVector3(0, 0, 0));
				fixed->setLinearUpperLimit(btVector3(0, 0, 0));
				fixed->setAngularLowerLimit(btVector3(0, 0, 0));
				fixed->setAngularUpperLimit(btVector3(0, 0, 0));

				con = fixed;

			}
			btAssert(con);
			if (con)
			{
				btJointFeedback* fb = new btJointFeedback();
				m_jointFeedback.push_back(fb);
				con->setJointFeedback(fb);

				m_dynamicsWorld->addConstraint(con, true);
			}
			prevBody = linkBody;

		}

	}

	if (1)
	{
		btVector3 groundHalfExtents(0.4, 0.0, 0.4);
		groundHalfExtents[upAxis] = 0.025f;
		btBoxShape* box = new btBoxShape(groundHalfExtents);
		box->initializePolyhedralFeatures();

		btTransform start; start.setIdentity();
		groundOrigin_target = btVector3(-0.4f, 3.9f, -0.77f);
		btVector3 basePosition = btVector3(-0.4f, 3.f, 0.f);
		btQuaternion groundOrn(btVector3(0, 1, 0), 0.25*SIMD_PI);


		start.setOrigin(groundOrigin_target);
		body = createRigidBody(0, start, box);

		body->setFriction(0);

	}
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

bool BasicExample::keyboardCallback(int key, int state)
{
	bool handled = false;
	if (state)
	{
		switch (key)
		{
		case B3G_HOME:
		{
			// b3Printf("Rest.\n");
			initState(this);
			break;
		}
		case B3G_LEFT_ARROW:
		{
			// b3Printf("left.\n");
			moveLeft(hinge);
			handled = true;
			break;

		}
		case B3G_RIGHT_ARROW:
		{
			// b3Printf("right.\n");
			moveRight(hinge);
			handled = true;
			break;
		}
		}
	}
	else
	{
		switch (key)
		{
		case B3G_LEFT_ARROW:
		case B3G_RIGHT_ARROW:
		{

			lockLiftHinge();
			handled = true;
			break;
		}
		default:

			break;
		}
	}
	return handled;
}

void BasicExample::lockLiftHinge(void)
{
	btScalar hingeAngle = hinge->getHingeAngle();
	btScalar lowLim = hinge->getLowerLimit();
	btScalar hiLim = hinge->getUpperLimit();
	hinge->enableAngularMotor(false, 0, 0);

	if (hingeAngle < lowLim)
	{
		//		m_liftHinge->setLimit(lowLim, lowLim + LIFT_EPS);
		hinge->setLimit(lowLim, lowLim);
	}
	else if (hingeAngle > hiLim)
	{
		//		m_liftHinge->setLimit(hiLim - LIFT_EPS, hiLim);
		hinge->setLimit(hiLim, hiLim);
	}
	else
	{
		//		m_liftHinge->setLimit(hingeAngle - LIFT_EPS, hingeAngle + LIFT_EPS);
		hinge->setLimit(hingeAngle, hingeAngle);
	}
	return;
}


CommonExampleInterface*    BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)