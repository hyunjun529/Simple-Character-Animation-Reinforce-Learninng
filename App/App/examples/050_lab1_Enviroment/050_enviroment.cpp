﻿#include "050_enviroment.h"
#include "../CommonEnviroment.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "CommonInterfaces/CommonRigidBodyBase.h"

#include <iostream>
#include "Actions.h"

struct Lab1Enviroment : public CommonRigidBodyBase
{
	
	float	Capsule_Width = 0.5f;
	float	Capsule_Radius = 0.12f;

	bool m_once;
	btAlignedObjectArray<btJointFeedback*> m_jointFeedback;
	btHingeConstraint* hinge_shader;
	btHingeConstraint* hinge_elbow;
	btRigidBody* linkBody[3];
	btVector3 groundOrigin_target;
	btRigidBody* body;
	btRigidBody* human_body;

	short collisionFilterGroup = short(btBroadphaseProxy::CharacterFilter);
	short collisionFilterMask = short(btBroadphaseProxy::AllFilter ^ (btBroadphaseProxy::CharacterFilter));

	btScalar F2T_distance_;
	float F2T_angle_;
	float eb_angle_;
	float sd_angle_;


	Lab1Enviroment(struct GUIHelperInterface* helper);
	virtual ~Lab1Enviroment();
	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);
	void lockLiftHinge(btHingeConstraint* hinge);
	virtual bool keyboardCallback(int key, int state);

	virtual void resetCamera() {
		float dist = 5;
		float pitch = 270;
		float yaw = 21;
		float targetPos[3] = { -1.34,3.4,-0.44 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}

	void initState(Lab1Enviroment* target) {
		target->m_guiHelper->removeAllGraphicsInstances();
		target->initPhysics();
	}

	void moveUpEb(btHingeConstraint *target) {
		target->setLimit(M_PI / 360.0f, M_PI / 1.2f);
		target->enableAngularMotor(true, 6.0, 4000.f);

	}

	void moveDownEb(btHingeConstraint *target) {
		target->setLimit(M_PI / 360.0f, M_PI / 1.2f);
		target->enableAngularMotor(true, -6.0, 4000.f);
	}

	void moveUpSd(btHingeConstraint *target) {
		target->setLimit(M_PI / 360.0f, M_PI / 1.2f);
		target->enableAngularMotor(true, 6.0, 4000.f);

	}

	void moveDownSd(btHingeConstraint *target) {
		target->setLimit(M_PI / 360.0f, M_PI / 1.2f);
		target->enableAngularMotor(true, -6.0, 4000.f);
	}

	float getAngleF2T() {
		return (atan((linkBody[2]->getCenterOfMassPosition().getZ() - body->getCenterOfMassPosition().getZ()) / (linkBody[2]->getCenterOfMassPosition().getY() - body->getCenterOfMassPosition().getY()))) * 180 / M_PI;
	}

	float getAngleSd() {
		return hinge_shader->getHingeAngle() / M_PI * 180;
	}

	float getAngleEb() {
		return hinge_elbow->getHingeAngle() / M_PI * 180;
	}
};

Lab1Enviroment::Lab1Enviroment(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper),
	m_once(true)
{
}

Lab1Enviroment::~Lab1Enviroment()
{
	for (int i = 0; i<m_jointFeedback.size(); i++)
	{
		delete m_jointFeedback[i];
	}
}

void Lab1Enviroment::lockLiftHinge(btHingeConstraint* hinge)
{
	btScalar hingeAngle = hinge->getHingeAngle();
	btScalar lowLim = hinge->getLowerLimit();
	btScalar hiLim = hinge->getUpperLimit();
	hinge->enableAngularMotor(false, 0, 0);

	if (hingeAngle < lowLim)
	{
		hinge->setLimit(lowLim, lowLim);
	}
	else if (hingeAngle > hiLim)
	{
		hinge->setLimit(hiLim, hiLim);
	}
	else
	{
		hinge->setLimit(hingeAngle, hingeAngle);
	}
	return;
}

void Lab1Enviroment::stepSimulation(float deltaTime)
{
	//get distance
	F2T_distance_ = sqrt(pow((body->getCenterOfMassPosition().getZ() - linkBody[2]->getCenterOfMassPosition().getZ()), 2) + pow((body->getCenterOfMassPosition().getY() - linkBody[2]->getCenterOfMassPosition().getY()), 2)) - 0.225;

	
	//collison check
	int collisionTarget = -1;
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

				//check the head or body
				if (F2T_distance_ <= sqrt(0.08f)) {
					collisionTarget = 0;
				}
				else {
					collisionTarget = 1;
				}
			}
		}
	}

	// elbow angle
	eb_angle_ = getAngleEb();

	// shoulder angle
	sd_angle_ = getAngleSd();

	// Fist to Target angle
	F2T_angle_ = getAngleF2T();
	
	//std::cout << "F2T_ang : " << F2T_angle_ << "\t" << "F2T_dis : " << F2T_distance_ << "\t";
	//std::cout << "eb_ang : " << eb_angle_ << "\t" << "sd_ang : " << sd_angle_ << "\t" << std::endl;

	//Joint Angular Speed 
	btVector3 angular_speed_sd = linkBody[0]->getAngularVelocity();
	btVector3 angular_speed_eb = linkBody[1]->getAngularVelocity();
	btVector3 hand_speed = linkBody[2]->getVelocityInLocalPoint(linkBody[2]->getCenterOfMassPosition());

	std::cout <<"SD_Angular Speed: "<< angular_speed_sd.getX() << "\t" << "EB_Angular Speed: " << angular_speed_eb.getX()<< "\t" << "hand Speed: " << hand_speed.getZ() << std::endl;

	if (collisionTarget == 0) {
		std::cout << "Collision Head" << std::endl;
	}
	if (collisionTarget == 1) {
		std::cout << "Collision Body" << std::endl;
	}

	m_dynamicsWorld->stepSimulation(1. / 240, 0);

	static int count = 0;
}

void Lab1Enviroment::initPhysics()
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
		int numLinks = 3;
		bool spherical = false;
		bool canSleep = false;
		bool selfCollide = false;

		btVector3 baseHalfExtents(0.4, 0.7, 0.1);
		btVector3 linkHalfExtents(0.05, 0.37, 0.1);

		btBoxShape* baseBox = new btBoxShape(baseHalfExtents);
		btVector3 basePosition = btVector3(-0.9f, 3.0f, 0.f);
		btTransform baseWorldTrans;
		baseWorldTrans.setIdentity();
		baseWorldTrans.setOrigin(basePosition);

		//init the base
		btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
		float baseMass = 0.f;
		float linkMass = 1.f;

		btRigidBody* base = createRigidBody(baseMass, baseWorldTrans, baseBox);
		m_dynamicsWorld->removeRigidBody(base);
		base->setDamping(0, 0);
		m_dynamicsWorld->addRigidBody(base, collisionFilterGroup, collisionFilterMask);

	/*	btBoxShape* linkBox1 = new btBoxShape(linkHalfExtents);
		btBoxShape* linkBox2 = new btBoxShape(linkHalfExtents);*/
		
		btCollisionShape* linkBox1 = new btCapsuleShape(Capsule_Radius, Capsule_Width);
		btCollisionShape* linkBox2 = new btCapsuleShape(Capsule_Radius, Capsule_Width);
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
			else if (i == 1)
			{
				colOb = linkBox2;
			}
			else
			{
				colOb = linkSphere;
			}

			linkBody[i] = createRigidBody(linkMass, linkTrans, colOb);
			m_dynamicsWorld->removeRigidBody(linkBody[i]);
			m_dynamicsWorld->addRigidBody(linkBody[i], collisionFilterGroup, collisionFilterMask);
			linkBody[i]->setDamping(0, 0);
			btTypedConstraint* con = 0;

			if (i == 0)
			{
				//create a hinge constraint
				btVector3 pivotInA(0.5, -linkHalfExtents[1] + 1.0, 0);
				btVector3 pivotInB(0, linkHalfExtents[1], 0);
				btVector3 axisInA(1, 0, 0);
				btVector3 axisInB(1, 0, 0);
				bool useReferenceA = true;
				hinge_shader = new btHingeConstraint(*prevBody, *linkBody[i],
					pivotInA, pivotInB,
					axisInA, axisInB, useReferenceA);
				hinge_shader->setLimit(M_PI / 0.24f, M_PI / 0.24f);
				m_dynamicsWorld->addConstraint(hinge_shader, true);
				con = hinge_shader;
			}
			else if (i == 1)
			{
				//create a hinge constraint
				btVector3 pivotInA(0, -linkHalfExtents[1], 0);
				btVector3 pivotInB(0, linkHalfExtents[1], 0);
				btVector3 axisInA(1, 0, 0);
				btVector3 axisInB(1, 0, 0);
				bool useReferenceA = true;
				hinge_elbow = new btHingeConstraint(*prevBody, *linkBody[i],
					pivotInA, pivotInB,
					axisInA, axisInB, useReferenceA);
				hinge_elbow->setLimit(M_PI / 1.92f, M_PI / 1.92f);
				m_dynamicsWorld->addConstraint(hinge_elbow, true);
				con = hinge_elbow;
			}
			else
			{
				btTransform pivotInA(btQuaternion::getIdentity(), btVector3(0, -radius, 0));						//par body's COM to cur body's COM offset
				btTransform pivotInB(btQuaternion::getIdentity(), btVector3(0, radius, 0));							//cur body's COM to cur body's PIV offset
				btGeneric6DofSpring2Constraint* fixed = new btGeneric6DofSpring2Constraint(*prevBody, *linkBody[i],
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
			prevBody = linkBody[i];
		}
	}


	if (1)
	{
		btVector3 groundHalfExtents(0.4, 0.0, 0.025);
		groundHalfExtents[upAxis] = 0.4f;
		btBoxShape* box = new btBoxShape(groundHalfExtents);
		box->initializePolyhedralFeatures();

		btTransform start; start.setIdentity();
		groundOrigin_target = btVector3(-0.4f, 4.0f, -1.25f);

		start.setOrigin(groundOrigin_target);
		body = createRigidBody(0, start, box);

		body->setFriction(0);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

bool Lab1Enviroment::keyboardCallback(int key, int state)
{
	bool handled = true;
	if (state)
	{
		switch (key)
		{
		case B3G_HOME:
		{
			initState(this);
			break;
		}
		case B3G_LEFT_ARROW:
		{
			moveUpEb(hinge_elbow);
			handled = true;
			break;
		}
		case B3G_RIGHT_ARROW:
		{
			moveDownEb(hinge_elbow);
			handled = true;
			break;
		}
		case B3G_UP_ARROW:
		{
			moveUpSd(hinge_shader);
			handled = true;
			break;
		}
		case B3G_DOWN_ARROW:
		{
			moveDownSd(hinge_shader);
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
			lockLiftHinge(hinge_elbow);
			handled = true;
			break;
		}
		case B3G_UP_ARROW:
		case B3G_DOWN_ARROW:
		{
			lockLiftHinge(hinge_shader);
			handled = true;
			break;
		}
		default:
			break;
		}
	}
	return handled;
}

CommonExampleInterface*    env_050(CommonExampleOptions& options)
{
	return new Lab1Enviroment(options.m_guiHelper);
}