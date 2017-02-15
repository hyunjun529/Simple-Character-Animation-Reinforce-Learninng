#include "052_enviroment.h"
#include "../CommonEnviroment.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "CommonInterfaces/CommonRigidBodyBase.h"

#include <iostream>
#include "Actions.h"
#include "Targets.h"
#include "Logger.h"
#include "Lab1ReinforcementLearning.h"

struct lab1Example2 : public CommonRigidBodyBase
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

	int selected_target = 0;
	float target_height[TARGET_SIZE] = {
		TARGET_0_HEIGHT,
		TARGET_1_HEIGHT,
		TARGET_2_HEIGHT,
		TARGET_3_HEIGHT,
		TARGET_4_HEIGHT,
		TARGET_5_HEIGHT,
	};

	float Fist_velocity;

	float F2T_distance_;
	float F2T_angle_;

	float sd_angle_;
	float sd_angular_velocity;

	float eb_angle_;
	float eb_angular_velocity;

	Lab1ReinforcementLearning rl_;

	Logger lg_;

	const int initStep = 100;
	const int maxStep = 500 + initStep;
	int cntStep;
	bool chkStudying;
	bool chkPrinting;


	lab1Example2(struct GUIHelperInterface* helper);
	virtual ~lab1Example2();
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


	// Controller evnets
	void initState(lab1Example2* target) {
		selected_target = (int)rand() % 6;
		target->m_guiHelper->removeAllGraphicsInstances();
		target->initPhysics();
	}

	void moveEbAngleUp(btHingeConstraint *target) {
		target->setLimit(M_PI / 360.0f, M_PI / 1.2f);
		target->enableAngularMotor(true, 6.0, 4000.f);
	}
	void moveEbAngleDown(btHingeConstraint *target) {
		target->setLimit(M_PI / 360.0f, M_PI / 1.2f);
		target->enableAngularMotor(true, -6.0, 4000.f);
	}
	void moveEbAngleStay(btHingeConstraint *target) {
		lockLiftHinge(target);
	}

	void moveSdAngleUp(btHingeConstraint *target) {
		target->setLimit(M_PI / 360.0f, M_PI / 1.2f);
		target->enableAngularMotor(true, 6.0, 4000.f);
	}
	void moveSdAngleDown(btHingeConstraint *target) {
		target->setLimit(M_PI / 360.0f, M_PI / 1.2f);
		target->enableAngularMotor(true, -6.0, 4000.f);
	}
	void moveSdAngleStay(btHingeConstraint *target) {
		lockLiftHinge(target);
	}


	// get States
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

lab1Example2::lab1Example2(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper),
	m_once(true)
{
	std::cout.precision(5);

	cntStep = 0;
	chkStudying = true;
	chkPrinting = true;
	lg_.fs_open("052_log.csv");

	/********************************************************************************************
	* start init rl_
	*********************************************************************************************/

	// h529 : 이전 게임의 기억을 가져감
	rl_.num_input_histories_ = 1;
	// h529 : 기억을 재생함, pool로 사용 중
	rl_.num_exp_replay_ = 0;
	// h529 : 현재 총 7개
	rl_.num_state_variables_ = 7;
	// h529 : 행동할 수 있는 Action의 개수, 현재 총 9개
	rl_.num_game_actions_ = 9;

	rl_.initialize();

	for (int h = 0; h < rl_.num_input_histories_; h++)
	{
		rl_.recordHistory(VectorND<float>(rl_.num_state_variables_), 0.0f, 2, VectorND<float>(rl_.num_game_actions_)); // choice 2 is stay
	}

	/********************************************************************************************
	* end init rl_
	*********************************************************************************************/
}

lab1Example2::~lab1Example2()
{
	for (int i = 0; i<m_jointFeedback.size(); i++)
	{
		delete m_jointFeedback[i];
	}
}

void lab1Example2::lockLiftHinge(btHingeConstraint* hinge)
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

void lab1Example2::stepSimulation(float deltaTime)
{
	if (cntStep < initStep) {

		cntStep++;

		m_dynamicsWorld->stepSimulation(1. / 240, 0);

		static int count = 0;

		return;
	}

	/********************************************************************************************
	* start set Action
	*********************************************************************************************/

	// set Vector
	rl_.forward();
	VectorND<float> output_vector_temp;
	rl_.nn_.copyOutputVectorTo(false, output_vector_temp);
	VectorND<float> output_target_temp;

	// set Action
	float dice = (chkStudying) ? (0.3f) : (0.0f);
	int action_ = rl_.nn_.getOutputIXEpsilonGreedy(dice);

	switch (action_) {

	case ACTION_SD_UP_EB_UP:
	{
		moveSdAngleUp(hinge_shoulder);
		moveEbAngleUp(hinge_elbow);
		break;
	}
	case ACTION_SD_UP_EB_DN:
	{
		moveSdAngleUp(hinge_shoulder);
		moveEbAngleDown(hinge_elbow);
		break;
	}
	case ACTION_SD_UP_EB_ST:
	{
		moveSdAngleUp(hinge_shoulder);
		moveEbAngleStay(hinge_elbow);
		break;
	}
	case ACTION_SD_DN_EB_UP:
	{
		moveSdAngleDown(hinge_shoulder);
		moveEbAngleUp(hinge_elbow);
		break;
	}
	case ACTION_SD_DN_EB_DN:
	{
		moveSdAngleDown(hinge_shoulder);
		moveEbAngleDown(hinge_elbow);
		break;
	}
	case ACTION_SD_DN_EB_ST:
	{
		moveSdAngleDown(hinge_shoulder);
		moveEbAngleStay(hinge_elbow);
		break;
	}
	case ACTION_SD_ST_EB_UP:
	{
		moveSdAngleStay(hinge_shoulder);
		moveEbAngleUp(hinge_elbow);
		break;
	}
	case ACTION_SD_ST_EB_DN:
	{
		moveSdAngleStay(hinge_shoulder);
		moveEbAngleDown(hinge_elbow);
		break;
	}
	case ACTION_SD_ST_EB_ST:
	{
		moveSdAngleStay(hinge_shoulder);
		moveEbAngleStay(hinge_elbow);
		break;
	}
	default: {}
	}

	/********************************************************************************************
	* end set Action
	*********************************************************************************************/


	/********************************************************************************************
	* start get State
	*********************************************************************************************/

	// get about Fist to Target
	F2T_distance_ = getF2TDistance();
	F2T_angle_ = getF2TAngle() / 180;

	// get Fist Velocity
	Fist_velocity = getFistVelocity();

	// get Shoulder states
	sd_angle_ = getSdAngle() / 180;
	sd_angular_velocity = getSdAngularVelocity();

	// get Elbow states
	eb_angle_ = getEbAngle() / 180;
	eb_angular_velocity = getEbAngularVelocity();

	
	//collison check
	bool chkCollision = false;
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
				chkCollision = true;
			}
		}
	}

	// calc Reward
	float weightStepEarly = (1 - ((float)cntStep / (((float)maxStep + (float)initStep ) * 1.25f)));
	float weightDistance = (1 - (F2T_distance_ / 1.5f));
	float weightAngle = (abs(F2T_angle_) * 2.0f);
	float weight_sd_Angle = (1 - ((abs(sd_angle_ * 180 - 90))) / 150);
	float weight_eb_Angle = (1 - (eb_angle_ * 180) / 150);
	float weight_fist_vel = (1 - (Fist_velocity / 60));
	float reward_ = weightDistance * weightAngle * weight_eb_Angle * weight_fist_vel;

	// set state VectorND
	VectorND<float> state_;
	state_.initialize(rl_.num_state_variables_, true);
	state_[0] = sd_angle_;
	state_[1] = sd_angular_velocity;
	state_[2] = eb_angle_;
	state_[3] = eb_angular_velocity;
	state_[4] = F2T_distance_;
	state_[5] = F2T_angle_;
	state_[6] = Fist_velocity;

	// Print current state
	if (chkPrinting) {
		std::cout << std::fixed << "selcted_action : " << action_ << "\t";
		//std::cout << std::fixed << "sd_ang : " << sd_angle_ << "\t" << "sd_ang_vel : " << sd_angular_velocity << "\t";
		//std::cout << std::fixed << "eb_ang : " << eb_angle_ << "\t" << "eb_ang_vel : " << eb_angular_velocity << "\t";
		
		//std::cout << std::fixed << "F2T_dis : " << F2T_distance_ << "\t";
		//std::cout << std::fixed << "F2T_ang : " << F2T_angle_ << "\t";
		//std::cout << std::fixed << "Fist_vel : " << Fist_velocity << "\t";
		
		std::cout << std::fixed << "weight_Fist_vel : " << weight_fist_vel << "\t";
		std::cout << std::fixed << "weight_F2T_Distance : " << weightDistance << "\t";
		std::cout << std::fixed << "weight_F2T_angle : " << weightAngle << "\t";
		
		std::cout << std::fixed << "reward : " << reward_ << "\t";
		std::cout << std::fixed << "current_step : " << cntStep << "\t";
		if (chkCollision) std::cout << "Collision !!!!!!!!!!!!\t";
		std::cout << std::endl;
	}

	// record history
	rl_.recordHistory(state_, reward_, action_, output_vector_temp);

	/********************************************************************************************
	* end get State
	*********************************************************************************************/


	/********************************************************************************************
	* start Training
	*********************************************************************************************/

	if ((chkCollision || cntStep > maxStep) && chkStudying) {

		// logging
		lg_.fout << sd_angle_ << ", " << eb_angle_ << ", " << F2T_angle_ << ", " << F2T_distance_ << ", " << selected_target << ", " << reward_ << ", " << cntStep << std::endl;

		// number of training 
		int tr_num = (chkCollision) ? (100) : (10);

		// sd
		for (int tr = 0; tr < tr_num; tr++)
			for (int m_tr = rl_.memory_.num_elements_ - 2; m_tr >= rl_.num_input_histories_; m_tr--)
			{
				// h529 : 방학숙제에서 발췌

				// stochastic training
				// h529 : 전체를 요약한 부분을 확률적으로 선택해서 학습하는 방법론
				// h529 : http://sanghyukchun.github.io/74/
				int m = rand() % (rl_.memory_.num_elements_ - 1 - rl_.num_input_histories_) + rl_.num_input_histories_;

				// memory index from end
				const int inv_m = m - (rl_.memory_.num_elements_ - 1);

				float Q_next = 0.0f;

				if (m != rl_.memory_.num_elements_ - 2) // if next is not the terminal state
				{
					// Q_next = ...;
					Q_next = rl_.memory_.q_values_array_[m + 1].getMaxValue();
				}

				float Q_target;
				// Q_target = ...;
				Q_target = Q_next + rl_.memory_.reward_array_[m];

				// forward propagation from previous inputs
				rl_.makeInputVectorFromHistory(inv_m - 1, rl_.old_input_vector_);
				rl_.nn_.setInputVector(rl_.old_input_vector_);
				for (int i = 0; i < 100; i++)
				{
					rl_.nn_.feedForward();
					rl_.nn_.copyOutputVectorTo(false, output_target_temp);

					// output_target_temp[...] = ...;
					output_target_temp[rl_.memory_.selected_array_[m]] = Q_target;

					rl_.nn_.propBackward(output_target_temp);
				}

				rl_.nn_.check();
			}

		// reset
		rl_.memory_.reset();

		// is really need?
		for (int h = 0; h < rl_.num_input_histories_; h++)
		{
			rl_.recordHistory(VectorND<float>(rl_.num_state_variables_), 0.0f, 2, VectorND<float>(rl_.num_game_actions_)); // choice 2 is stay
		}
		cntStep = 0;

		initState(this);
	}

	if (chkCollision && chkStudying) {
		// reset
		rl_.memory_.reset();

		// is really need?
		for (int h = 0; h < rl_.num_input_histories_; h++)
		{
			rl_.recordHistory(VectorND<float>(rl_.num_state_variables_), 0.0f, 2, VectorND<float>(rl_.num_game_actions_)); // choice 2 is stay
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

void lab1Example2::initPhysics()
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

	btVector3 basePosition_origen = btVector3(-0.4f, 4.f, 0.f);
	btTransform baseWorldTrans_origen;
	baseWorldTrans_origen.setIdentity();
	baseWorldTrans_origen.setOrigin(basePosition_origen);

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

	btQuaternion orn[3];
	orn[0] = btQuaternion(btVector3(1, 0, 0), 0.25*3.1415926538);
	orn[1] = btQuaternion(btVector3(1, 0, 0), 0.75*3.1415926538);
	orn[2] = btQuaternion(btVector3(1, 0, 0), 0.25*3.1415926538);

	for (int i = 0; i<numLinks; i++)
	{
		btTransform linkTrans;
		linkTrans = baseWorldTrans_origen;

		linkTrans.setOrigin(basePosition_origen - btVector3(0, linkHalfExtents[1] * 2.f*(i / 4 + 1) - 0.1, i*0.5 + 0.2));
		linkTrans.setRotation(orn[i]);

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
			hinge_shoulder->setLimit(M_PI / 0.24f, M_PI / 0.24f);
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

	// Target Position
	btSphereShape* linkSphere_1 = new btSphereShape(radius);
	btTransform start; start.setIdentity();
	groundOrigin_target = btVector3(-0.4f, target_height[selected_target], -1.6f);
	start.setOrigin(groundOrigin_target);
	body = createRigidBody(0, start, linkSphere_1);
	body->setFriction(0);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

bool lab1Example2::keyboardCallback(int key, int state)
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

CommonExampleInterface*    env_052(CommonExampleOptions& options)
{
	return new lab1Example2(options.m_guiHelper);
}