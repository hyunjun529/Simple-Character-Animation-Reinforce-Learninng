#include "021_enviroment.h"
#include "../CommonEnviroment.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "CommonInterfaces/CommonRigidBodyBase.h"

#include <iostream>
#include "Actions.h"
#include "Logger.h"
#include "ArmReinforcementLearning.h"

ArmReinforcementLearning rl_sd_021_;
ArmReinforcementLearning rl_eb_021_;

struct lab0AddAngle : public CommonRigidBodyBase
{
	bool m_once;
	btAlignedObjectArray<btJointFeedback*> m_jointFeedback;
	btHingeConstraint* hinge_shoulder;
	btHingeConstraint* hinge_elbow;
	btRigidBody* linkBody;
	btVector3 groundOrigin_target;
	btRigidBody* body;
	btRigidBody* human_body;

	short collisionFilterGroup = short(btBroadphaseProxy::CharacterFilter);
	short collisionFilterMask = short(btBroadphaseProxy::AllFilter ^ (btBroadphaseProxy::CharacterFilter));

	btScalar F2T_distance_;
	float F2T_angle_;
	float eb_angle_;
	float sd_angle_;

	const int maxStep = 500;
	int cntStep;
	bool chkStudying;
	bool chkPrinting;
	Logger lg_;

	lab0AddAngle(struct GUIHelperInterface* helper);
	virtual ~lab0AddAngle();
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

	void initState(lab0AddAngle* target) {
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
		return (atan((linkBody->getCenterOfMassPosition().getZ() - body->getCenterOfMassPosition().getZ()) / (linkBody->getCenterOfMassPosition().getY() - body->getCenterOfMassPosition().getY()))) * 180 / M_PI;
	}

	float getAngleEb() {
		return hinge_shoulder->getHingeAngle() / M_PI * 180;;
	}

	float getAngleSd() {
		return hinge_elbow->getHingeAngle() / M_PI * 180;
	}
};

lab0AddAngle::lab0AddAngle(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper),
	m_once(true)
{
	cntStep = 0;
	chkStudying = true;
	chkPrinting = true;
	lg_.fs_open("021_log.csv");

	std::cout.precision(5);

	/********************************************************************************************
	* start init rl_
	*********************************************************************************************/

	// h529 : 이전 게임의 기억을 가져감
	rl_sd_021_.num_input_histories_ = 1;
	// h529 : 기억을 재생함, pool로 사용 중
	rl_sd_021_.num_exp_replay_ = 0;
	// h529 : eb_angle, sd_angle, F2T_distance, F2T_angle
	rl_sd_021_.num_state_variables_ = 4;
	// h529 : 행동할 수 있는 Action의 개수 현재 좌, 우 2개
	rl_sd_021_.num_game_actions_ = 3;

	rl_sd_021_.initialize();

	for (int h = 0; h < rl_sd_021_.num_input_histories_; h++)
	{
		rl_sd_021_.recordHistory(VectorND<float>(4), 0.0f, 2, VectorND<float>(3)); // choice 2 is stay
	}

	// h529 : 이전 게임의 기억을 가져감
	rl_eb_021_.num_input_histories_ = 1;
	// h529 : 기억을 재생함, pool로 사용 중
	rl_eb_021_.num_exp_replay_ = 0;
	// h529 : eb_angle, sd_angle, F2T_distance, F2T_angle
	rl_eb_021_.num_state_variables_ = 4;
	// h529 : 행동할 수 있는 Action의 개수 현재 좌, 우 2개
	rl_eb_021_.num_game_actions_ = 3;

	rl_eb_021_.initialize();

	for (int h = 0; h < rl_eb_021_.num_input_histories_; h++)
	{
		rl_eb_021_.recordHistory(VectorND<float>(4), 0.0f, 2, VectorND<float>(3)); // choice 2 is stay
	}

	/********************************************************************************************
	* end init rl_
	*********************************************************************************************/
}

lab0AddAngle::~lab0AddAngle()
{
	for (int i = 0; i<m_jointFeedback.size(); i++)
	{
		delete m_jointFeedback[i];
	}
}

void lab0AddAngle::lockLiftHinge(btHingeConstraint* hinge)
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

void lab0AddAngle::stepSimulation(float deltaTime)
{
	bool chkCollision = false;


	/********************************************************************************************
	* start set Action
	*********************************************************************************************/

	// set Vector
	rl_sd_021_.forward();
	VectorND<float> output_vector_temp;
	rl_sd_021_.nn_.copyOutputVectorTo(false, output_vector_temp);
	VectorND<float> output_target_temp;

	rl_eb_021_.forward();
	VectorND<float> output_vector_temp_eb;
	rl_eb_021_.nn_.copyOutputVectorTo(false, output_vector_temp_eb);
	VectorND<float> output_target_temp_eb;

	// set Action
	float dice = (chkStudying) ? (0.3f) : (0.0f);
	int action_sd = rl_sd_021_.nn_.getOutputIXEpsilonGreedy(dice);
	int action_eb = rl_eb_021_.nn_.getOutputIXEpsilonGreedy(dice);

	switch (action_sd) {
	case ACTION_SHOULDER_UP:
	{
		moveUpSd(hinge_shoulder);
		break;
	}
	case ACTION_SHOULDER_DOWN:
	{
		moveDownSd(hinge_shoulder);
		break;
	}
	case ACTION_SHOULDER_STAY:
	{
		lockLiftHinge(hinge_shoulder);
		break;
	}
	default: {}
	}

	switch (action_eb) {
	case ACTION_ELBOW_UP:
	{
		moveUpEb(hinge_elbow);
		break;
	}
	case ACTION_ELBOW_DOWN:
	{
		moveDownEb(hinge_elbow);
		break;
	}
	case ACTION_ELBOW_STAY:
	{
		lockLiftHinge(hinge_elbow);
		break;
	}
	default: {}
	}

	/********************************************************************************************
	* end set Action
	*********************************************************************************************/


	/********************************************************************************************
	* start get States
	*********************************************************************************************/

	//get distance
	F2T_distance_ = sqrt(pow((body->getCenterOfMassPosition().getZ() - linkBody->getCenterOfMassPosition().getZ()), 2) + pow((body->getCenterOfMassPosition().getY() - linkBody->getCenterOfMassPosition().getY()), 2)) - 0.225;

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

				chkCollision = true;
			}
		}
	}

	// elbow angle
	eb_angle_ = getAngleEb();

	// shoulder angle
	sd_angle_ = getAngleSd();

	// Fist to Target angle
	F2T_angle_ = getAngleF2T();
	
	// calc Reward
	float weightStepEarly = (1 - ((float)cntStep / ((float)maxStep * 1.25f)));
	float weightDistance = (1 - (F2T_distance_ / 2.5f));
	float weightAngle = (1 - (abs(F2T_angle_) / 90.0f));
	float reward_ = weightDistance;

	// set state VectorND
	VectorND<float> state_;
	state_.initialize(4, true);
	state_[0] = sd_angle_;
	state_[1] = eb_angle_;
	state_[2] = F2T_angle_;
	state_[3] = F2T_distance_;

	rl_sd_021_.recordHistory(state_, reward_, action_sd, output_vector_temp);
	rl_eb_021_.recordHistory(state_, reward_, action_eb, output_vector_temp_eb);

	if (chkPrinting) {
		std::cout << std::fixed << "Action_sd : " << action_sd << "\t" << "Action_eb : " << action_eb << "\t";
		std::cout << std::fixed << "F2T_ang : " << F2T_angle_ << "\t" << "F2T_dis : " << F2T_distance_ << "\t";
		std::cout << std::fixed << "eb_ang : " << eb_angle_ << "\t" << "sd_ang : " << sd_angle_ << "\t";
		std::cout << std::fixed << "reward : " << reward_ << std::endl;
	}

	/********************************************************************************************
	* end get States
	*********************************************************************************************/


	/********************************************************************************************
	* start Training
	*********************************************************************************************/

	if ((chkCollision || cntStep > maxStep) && chkStudying) {
		
		// compensation
		for (float &val : rl_sd_021_.memory_.reward_array_) {
			val *= weightStepEarly;
		}
		for (float &val : rl_eb_021_.memory_.reward_array_) {
			val *= weightStepEarly;
		}

		// logging
		lg_.fout << weightStepEarly << ", " << F2T_angle_ << ", " << F2T_distance_ << ", " << reward_ << ", " << cntStep << std::endl;

		// number of training 
		int tr_num = (chkCollision)?(100):(10);

		// sd
		for (int tr = 0; tr < tr_num; tr++)
			for (int m_tr = rl_sd_021_.memory_.num_elements_ - 2; m_tr >= rl_sd_021_.num_input_histories_; m_tr--)
			{
				// h529 : 방학숙제에서 발췌

				// stochastic training
				// h529 : 전체를 요약한 부분을 확률적으로 선택해서 학습하는 방법론
				// h529 : http://sanghyukchun.github.io/74/
				int m = rand() % (rl_sd_021_.memory_.num_elements_ - 1 - rl_sd_021_.num_input_histories_) + rl_sd_021_.num_input_histories_;

				// memory index from end
				const int inv_m = m - (rl_sd_021_.memory_.num_elements_ - 1);

				float Q_next = 0.0f;

				if (m != rl_sd_021_.memory_.num_elements_ - 2) // if next is not the terminal state
				{
					// Q_next = ...;
					Q_next = rl_sd_021_.memory_.q_values_array_[m + 1].getMaxValue();
				}

				float Q_target;
				// Q_target = ...;
				Q_target = Q_next + rl_sd_021_.memory_.reward_array_[m];

				// forward propagation from previous inputs
				rl_sd_021_.makeInputVectorFromHistory(inv_m - 1, rl_sd_021_.old_input_vector_);
				rl_sd_021_.nn_.setInputVector(rl_sd_021_.old_input_vector_);
				for (int i = 0; i < 100; i++)
				{
					rl_sd_021_.nn_.feedForward();
					rl_sd_021_.nn_.copyOutputVectorTo(false, output_target_temp);

					// output_target_temp[...] = ...;
					output_target_temp[rl_sd_021_.memory_.selected_array_[m]] = Q_target;

					rl_sd_021_.nn_.propBackward(output_target_temp);
				}

				rl_sd_021_.nn_.check();
			}

		// eb
		for (int tr = 0; tr < tr_num; tr++)
			for (int m_tr = rl_eb_021_.memory_.num_elements_ - 2; m_tr >= rl_eb_021_.num_input_histories_; m_tr--)
			{
				// h529 : 방학숙제에서 발췌

				// stochastic training
				// h529 : 전체를 요약한 부분을 확률적으로 선택해서 학습하는 방법론
				// h529 : http://sanghyukchun.github.io/74/
				int m = rand() % (rl_eb_021_.memory_.num_elements_ - 1 - rl_eb_021_.num_input_histories_) + rl_eb_021_.num_input_histories_;

				// memory index from end
				const int inv_m = m - (rl_eb_021_.memory_.num_elements_ - 1);

				float Q_next = 0.0f;

				if (m != rl_eb_021_.memory_.num_elements_ - 2) // if next is not the terminal state
				{
					// Q_next = ...;
					Q_next = rl_eb_021_.memory_.q_values_array_[m + 1].getMaxValue();
				}

				float Q_target;
				// Q_target = ...;
				Q_target = Q_next + rl_eb_021_.memory_.reward_array_[m];

				// forward propagation from previous inputs
				rl_eb_021_.makeInputVectorFromHistory(inv_m - 1, rl_eb_021_.old_input_vector_);
				rl_eb_021_.nn_.setInputVector(rl_eb_021_.old_input_vector_);
				for (int i = 0; i < 100; i++)
				{
					rl_eb_021_.nn_.feedForward();
					rl_eb_021_.nn_.copyOutputVectorTo(false, output_target_temp_eb);

					// output_target_temp[...] = ...;
					output_target_temp_eb[rl_eb_021_.memory_.selected_array_[m]] = Q_target;

					rl_eb_021_.nn_.propBackward(output_target_temp_eb);
				}

				rl_eb_021_.nn_.check();
			}

		// reset
		rl_sd_021_.memory_.reset();
		rl_eb_021_.memory_.reset();

		// is really need?
		for (int h = 0; h < rl_sd_021_.num_input_histories_; h++)
		{
			rl_sd_021_.recordHistory(VectorND<float>(4), 0.0f, 2, VectorND<float>(3)); // choice 2 is stay
		}
		for (int h = 0; h < rl_eb_021_.num_input_histories_; h++)
		{
			rl_eb_021_.recordHistory(VectorND<float>(4), 0.0f, 2, VectorND<float>(3)); // choice 2 is stay
		}

		cntStep = 0;

		initState(this);
	}

	/********************************************************************************************
	* end Training
	*********************************************************************************************/

	cntStep++;

	m_dynamicsWorld->stepSimulation(1. / 240, 0);

	static int count = 0;
}

void lab0AddAngle::initPhysics()
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
		btBoxShape* linkBox2 = new btBoxShape(linkHalfExtents);
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
				hinge_shoulder = new btHingeConstraint(*prevBody, *linkBody,
					pivotInA, pivotInB,
					axisInA, axisInB, useReferenceA);
				hinge_shoulder->setLimit(0.0f, 0.0f);
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
				hinge_elbow = new btHingeConstraint(*prevBody, *linkBody,
					pivotInA, pivotInB,
					axisInA, axisInB, useReferenceA);
				hinge_elbow->setLimit(0.0f, 0.0f);
				m_dynamicsWorld->addConstraint(hinge_elbow, true);
				con = hinge_elbow;
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

		btSphereShape* linkSphere_1 = new btSphereShape(radius);

		btTransform start; start.setIdentity();
		groundOrigin_target = btVector3(-0.4f, 4.0f, -1.6f);

		start.setOrigin(groundOrigin_target);
		body = createRigidBody(0, start, linkSphere_1);

		body->setFriction(0);

	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

bool lab0AddAngle::keyboardCallback(int key, int state)
{
	bool handled = true;
	if (state)
	{
		switch (key)
		{
		case B3G_INSERT:
		{
			chkPrinting = (chkPrinting) ? (false) : (true);
			break;
		}
		case B3G_END:
		{
			chkStudying = (chkStudying) ? (false) : (true);
			break;
		}
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
			moveUpSd(hinge_shoulder);
			handled = true;
			break;
		}
		case B3G_DOWN_ARROW:
		{
			moveDownSd(hinge_shoulder);
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

CommonExampleInterface*    env_021(CommonExampleOptions& options)
{
	return new lab0AddAngle(options.m_guiHelper);
}