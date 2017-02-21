#include "210_enviroment.h"
#include "../CommonEnviroment.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "CommonInterfaces/CommonRigidBodyBase.h"

#include <NeuralNetwork.h>
#include <VectorND.h>

#include <iostream>
#include "Actions.h"
#include "Targets.h"

struct LabF210 : public CommonRigidBodyBase
{
	float Capsule_Width = 0.5f;
	float Capsule_Radius = 0.12f;

	bool m_once;
	btAlignedObjectArray<btJointFeedback*> m_jointFeedback;
	btHingeConstraint* hinge_shoulder;
	btHingeConstraint* hinge_elbow;
	btRigidBody* linkBody[3];
	btVector3 groundOrigin_target;
	btRigidBody* body;

	short collisionFilterGroup = short(btBroadphaseProxy::CharacterFilter);
	short collisionFilterMask = short(btBroadphaseProxy::AllFilter ^ (btBroadphaseProxy::CharacterFilter));

	float target_height;

	float Fist_velocity;
	float F2T_distance_;
	float F2T_angle_;

	float sd_angle_;
	float sd_current_angular_velocity;
	float sd_target_angular_velocity;
	const float sd_max_angular_velocity = 3.f;

	float eb_angle_;
	float eb_current_angular_velocity;
	float eb_target_angular_velocity;
	const float eb_max_angular_velocity = 3.f;

	NeuralNetwork nn_;
	int num_state_variables_;
	int num_game_actions_;


	LabF210(struct GUIHelperInterface* helper);
	virtual ~LabF210();
	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);
	virtual bool keyboardCallback(int key, int state);

	virtual void resetCamera() {
		float dist = 5;
		float pitch = 270;
		float yaw = 21;
		float targetPos[3] = { -1.34,3.4,-0.44 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}


	// Controller evnets
	void initState() {
		sd_target_angular_velocity = 0.f;
		eb_target_angular_velocity = 0.f;
		m_guiHelper->removeAllGraphicsInstances();
		initPhysics();
	}
	void initTarget() {
		btSphereShape* linkSphere_1 = new btSphereShape(radius);
		btTransform start;
		start.setIdentity();
		groundOrigin_target = btVector3(-0.4f, target_height, -1.6f);
		start.setOrigin(groundOrigin_target);
		body->setWorldTransform(start);

		m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
		m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	}
	void lockLiftHinge(btHingeConstraint* hinge) {
		btScalar hingeAngle = hinge->getHingeAngle();
		btScalar lowLim = hinge->getLowerLimit();
		btScalar hiLim = hinge->getUpperLimit();
		hinge->enableAngularMotor(false, 0, 0);

		if (hingeAngle < lowLim) {
			hinge->setLimit(lowLim, lowLim);
		}
		else if (hingeAngle > hiLim) {
			hinge->setLimit(hiLim, hiLim);
		}
		else {
			hinge->setLimit(hingeAngle, hingeAngle);
		}
		return;
	}
	float getRandomTargetPosition() {
		return (TARGET_HEIGHT_MAX - TARGET_HEIGHT_MIN) * ((float)rand() / (float)RAND_MAX) + TARGET_HEIGHT_MIN;
	}

	void moveEbAngle(btHingeConstraint *target) {
		target->setLimit(M_PI / 360.0f, M_PI / 1.2f);
		target->enableAngularMotor(true, eb_target_angular_velocity, 4000.f);
	}
	void moveEbAngleUp(btHingeConstraint *target) {
		eb_target_angular_velocity += 0.5f;
	}
	void moveEbAngleDown(btHingeConstraint *target) {
		eb_target_angular_velocity -= 0.5f;
	}
	void moveEbAngleStay(btHingeConstraint *target) {
		//lockLiftHinge(target);
	}

	void moveSdAngle(btHingeConstraint *target) {
		target->setLimit(M_PI / 360.0f, M_PI / 1.2f);
		target->enableAngularMotor(true, sd_target_angular_velocity, 4000.f);
	}
	void moveSdAngleUp(btHingeConstraint *target) {
		if (sd_target_angular_velocity < sd_max_angular_velocity)
			sd_target_angular_velocity += 0.5f;
	}
	void moveSdAngleDown(btHingeConstraint *target) {
		if (sd_target_angular_velocity > sd_max_angular_velocity * -1)
			sd_target_angular_velocity -= 0.5f;
	}
	void moveSdAngleStay(btHingeConstraint *target) {
		//lockLiftHinge(target);
	}


	float getF2TDistance() {
		return sqrt(pow((body->getCenterOfMassPosition().getZ() - linkBody[2]->getCenterOfMassPosition().getZ()), 2) + pow((body->getCenterOfMassPosition().getY() - linkBody[2]->getCenterOfMassPosition().getY()), 2)) - 0.225;
	}
	float getF2TAngle() {
		return (atan((linkBody[2]->getCenterOfMassPosition().getZ() - body->getCenterOfMassPosition().getZ()) / (linkBody[2]->getCenterOfMassPosition().getY() - body->getCenterOfMassPosition().getY()))) * 180 / M_PI;
	}
	float getFistVelocity() {
		return abs(linkBody[2]->getVelocityInLocalPoint(linkBody[2]->getCenterOfMassPosition()).getZ());
	}

	float getSdAngle() {
		return hinge_shoulder->getHingeAngle() / M_PI * 180;
	}
	float getSdAngularVelocity() {
		return abs(linkBody[0]->getAngularVelocity().getX());
	}

	float getEbAngle() {
		return hinge_elbow->getHingeAngle() / M_PI * 180;
	}
	float getEbAngularVelocity() {
		return abs(linkBody[1]->getAngularVelocity().getX());
	}
};

LabF210::LabF210(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper),
	m_once(true)
{
	std::cout.precision(5);
	target_height = getRandomTargetPosition();
	sd_target_angular_velocity = 0.f;
	eb_target_angular_velocity = 0.f;

	const int num_hidden_layers = 1;
	num_state_variables_ = 4;
	num_game_actions_ = 3;

	nn_.initialize(num_state_variables_, num_game_actions_, num_hidden_layers);

	for (int i = 0; i <= num_hidden_layers + 1; i++)
		nn_.layers_[i].act_type_ = LayerBase::ReLU;

	nn_.eta_ = 1e-5;
	nn_.alpha_ = 0.9f;
}

LabF210::~LabF210()
{
	for (int i = 0; i<m_jointFeedback.size(); i++)
	{
		delete m_jointFeedback[i];
	}
}

void LabF210::stepSimulation(float deltaTime)
{
	/***************************************************************************************************/
	// start Set State
	/***************************************************************************************************/

	// get about Fist to Target
	F2T_distance_ = getF2TDistance();
	F2T_angle_ = getF2TAngle() / 180;

	// get Fist Velocity
	Fist_velocity = getFistVelocity();

	// get Shoulder states
	sd_angle_ = getSdAngle() / 180;
	sd_current_angular_velocity = getSdAngularVelocity();

	// get Elbow states
	eb_angle_ = getEbAngle() / 180;
	eb_current_angular_velocity = getEbAngularVelocity();


	//collison check
	bool collisionTarget = false;
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
				collisionTarget = true;
			}
		}
	}

	// set Vector
	VectorND<float> input;
	input.initialize(num_state_variables_);
	input[0] = F2T_distance_;
	input[1] = F2T_angle_;
	input[2] = sd_angle_;
	input[3] = sd_current_angular_velocity;

	nn_.setInputVector(input);
	nn_.feedForward();

	VectorND<float> output;
	nn_.copyOutputVectorTo(false, output);

	/***************************************************************************************************/
	// End Set State
	/***************************************************************************************************/


	/***************************************************************************************************/
	// start Set Action
	/***************************************************************************************************/

	float dice = (true) ? (0.1f) : (0.0f);
	int action_ = nn_.getOutputIXEpsilonGreedy(dice);

	switch (action_) {
	case ACTION_SHOULDER_UP: {
		moveSdAngleUp(hinge_shoulder);
		break;
	}
	case ACTION_SHOULDER_DOWN: {
		moveSdAngleDown(hinge_shoulder);
		break;
	}
	case ACTION_SHOULDER_STAY: {
		moveSdAngleStay(hinge_shoulder);
		break;
	}
	default:
		action_ = ACTION_SHOULDER_STAY;
	}

	moveSdAngle(hinge_shoulder);
	//moveEbAngle(hinge_elbow);

	/***************************************************************************************************/
	// End Set Action
	/***************************************************************************************************/


	/***************************************************************************************************/
	// start Training
	/***************************************************************************************************/

	// set reward, reward_vector
	float cost_F2T_Distance = 1.f - (F2T_distance_ / 2.4f);
	float reward_ = cost_F2T_Distance;

	VectorND<float> reward_vector(output);

	for (int d = 0; d < reward_vector.num_dimension_; d++) {
		if (action_ == d) {
			reward_vector[d] = reward_ > 0.01f ? 0.999 : 0.001;
		}
		else {
			reward_vector[d] = reward_vector[d] < 0.001 ? 0.001 : reward_vector[d];
		}
	}

	// back to the future
	int num_tr = (collisionTarget) ? (10) : (1);
	for (int i = 0; i < num_tr; i++) {
		nn_.propBackward(reward_vector);
	}

	/***************************************************************************************************/
	// End Training
	/***************************************************************************************************/
	

	/***************************************************************************************************/
	// start Printing current condition
	/***************************************************************************************************/

	// Print current state
	std::cout << std::fixed << "action : " << action_ << "\t";
	std::cout << std::fixed << "sd_ang : " << sd_angle_ << "\t";
	//std::cout << std::fixed << "sd_ang_vel : " << sd_current_angular_velocity << "\t";
	std::cout << std::fixed << "sd_ang_vel : " << sd_target_angular_velocity << "\t";
	//std::cout << std::fixed << "eb_ang : " << eb_angle_ << "\t" << "eb_ang_vel : " << eb_current_angular_velocity << "\t";
	std::cout << std::fixed << "F2T_dis : " << F2T_distance_ << "\t";
	std::cout << std::fixed << "F2T_ang : " << F2T_angle_ << "\t";
	std::cout << std::fixed << "Fist_vel : " << Fist_velocity << "\t";
	std::cout << std::fixed << "reward : " << reward_ << "\t";
	if (collisionTarget) std::cout << "\tCollision !!!!!!!!!!!!";
	std::cout << std::endl;

	/***************************************************************************************************/
	// End Printing current condition
	/***************************************************************************************************/


	/***************************************************************************************************/
	// start process Step
	/***************************************************************************************************/

	// Reset Target Position
	if (collisionTarget) {
		target_height = getRandomTargetPosition();

		//initState();
		initTarget();
	}

	// end point
	if (Fist_velocity < 0.01f && sd_max_angular_velocity <= abs(sd_target_angular_velocity)) {
		initState();
	}

	// step by step
	m_dynamicsWorld->stepSimulation(1. / 240, 0);

	/***************************************************************************************************/
	// End process Step
	/***************************************************************************************************/
}

void LabF210::initPhysics()
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

	int numLinks = 3;
	bool spherical = false;
	bool canSleep = false;
	bool selfCollide = false;

	btVector3 baseHalfExtents(0.4f, 0.7f, 0.1f);
	btVector3 linkHalfExtents(0.05f, 0.37f, 0.1f);

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
			btVector3 pivotInA(0.5, -linkHalfExtents[1] + 1.0f, 0);
			btVector3 pivotInB(0, linkHalfExtents[1], 0);
			btVector3 axisInA(1, 0, 0);
			btVector3 axisInB(1, 0, 0);
			bool useReferenceA = true;
			hinge_shoulder = new btHingeConstraint(*prevBody, *linkBody[i],
				pivotInA, pivotInB,
				axisInA, axisInB, useReferenceA);
			hinge_shoulder->setLimit(0.f, 0.f);
			//hinge_shoulder->setLimit(M_PI / 0.48f, M_PI / 0.48f);
			m_dynamicsWorld->addConstraint(hinge_shoulder, true);
			con = hinge_shoulder;
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
			hinge_elbow->setLimit(0.f, 0.f);
			//hinge_elbow->setLimit(M_PI / 1.3f, M_PI / 1.3f);
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

	// Target Position
	btSphereShape* linkSphere_1 = new btSphereShape(radius);
	btTransform start; start.setIdentity();
	groundOrigin_target = btVector3(-0.4f, target_height, -1.6f);
	start.setOrigin(groundOrigin_target);
	body = createRigidBody(0, start, linkSphere_1);
	body->setFriction(0);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

bool LabF210::keyboardCallback(int key, int state)
{
	bool handled = true;
	if (state)
	{
		switch (key)
		{
		case B3G_HOME:
		{
			initState();
			break;
		}
		/*
		case B3G_LEFT_ARROW:
		{
			moveEbAngleUp(hinge_elbow);
			handled = true;
			break;
		}
		case B3G_RIGHT_ARROW:
		{
			moveEbAngleDown(hinge_elbow);
			handled = true;
			break;
		}
		*/
		case B3G_UP_ARROW:
		{
			moveSdAngleUp(hinge_shoulder);
			handled = true;
			break;
		}
		case B3G_DOWN_ARROW:
		{
			moveSdAngleDown(hinge_shoulder);
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
			lockLiftHinge(hinge_shoulder);
			handled = true;
			break;
		}
		default:
			break;
		}
	}
	return handled;
}

CommonExampleInterface*    env_210(CommonExampleOptions& options)
{
	return new LabF210(options.m_guiHelper);
}