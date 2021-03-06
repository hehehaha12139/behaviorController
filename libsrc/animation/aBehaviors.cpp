#include "aBehaviors.h"

#include <math.h>
#include "GL/glew.h"
#include "GL/glut.h"

// Base Behavior
///////////////////////////////////////////////////////////////////////////////
Behavior::Behavior()
{
}

Behavior::Behavior( char* name) 
{
	m_name = name;
	m_pTarget = NULL;
}

Behavior::Behavior( Behavior& orig) 
{
	m_name = orig.m_name;
	m_pTarget = NULL;
}

string& Behavior::GetName() 
{
    return m_name;
}

// Behaviors derived from Behavior
//----------------------------------------------------------------------------//
// Seek behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Seek returns a maximum velocity towards the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position


Seek::Seek( AJoint* target) 
{
	m_name = "seek";
	m_pTarget = target;
}

Seek::Seek( Seek& orig) 
{
	m_name = "seek";
	m_pTarget = orig.m_pTarget;
}


Seek::~Seek()
{
}

vec3 Seek::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	vec3 Xaxis, Yaxis, Zaxis;
	vec3 error = targetPos - actorPos;
	Yaxis = vec3(0.0, 1.0, 0.0);

	mat3 zRot = mat3();
	zRot.FromAxisAngle(Yaxis, actor->getOrientation()[_Y]);

	Zaxis = zRot * vec3(0.0, 0.0, 1.0);

	Xaxis = Yaxis.Cross(Zaxis);
	
	double theta = atan2(Zaxis[0], Zaxis[2]);

	float errorXLength = error * Xaxis / Xaxis.Length();
	float errorZLength = error * Zaxis / Zaxis.Length();

	double transAngle = atan2(errorXLength, errorZLength);

	transAngle = theta + transAngle;

	mat3 velocityRot = mat3();
	velocityRot.FromAxisAngle(vec3(0.0, 1.0, 0.0), transAngle);

	Vdesired = actor->gMaxSpeed * velocityRot * vec3(0.0, 0.0, 1.0);

	actor->gVelKv = 10;
	actor->gOriKp = 256;
	actor->gOriKv = 32;
	

	return Vdesired;
}


// Flee behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Flee calculates a a maximum velocity away from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position

Flee::Flee( AJoint* target) 
{
	m_name = "flee";
	m_pTarget = target;
}

Flee::Flee( Flee& orig) 
{
	m_name = "flee";
	m_pTarget = orig.m_pTarget;
}

Flee::~Flee()
{
}

vec3 Flee::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	vec3 Xaxis, Yaxis, Zaxis;
	vec3 error = targetPos - actorPos;
	Yaxis = vec3(0.0, 1.0, 0.0);

	if (actor->getVelocity().Length() == 0)
	{
		Zaxis = vec3(0.0, 0.0, 1.0);
	}
	else
	{
		Zaxis = actor->getVelocity().Normalize();
	}


	Xaxis = Yaxis.Cross(Zaxis);

	double theta = atan2(Zaxis[0], Zaxis[2]);

	float errorXLength = error * Xaxis / Xaxis.Length();
	float errorZLength = error * Zaxis / Zaxis.Length();

	double transAngle = atan2(errorXLength, errorZLength);

	transAngle = theta + transAngle + M_PI;

	mat3 velocityRot = mat3();
	velocityRot.FromAxisAngle(vec3(0.0, 1.0, 0.0), transAngle);

	Vdesired = actor->gMaxSpeed * velocityRot * vec3(0.0, 0.0, 1.0);

	actor->gVelKv = 10;
	actor->gOriKp = 256;
	actor->gOriKv = 32;


	return Vdesired;
}

// Arrival behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// the actors distance from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Arrival strength is in BehavioralController::KArrival


Arrival::Arrival( AJoint* target) 
{
	m_name = "arrival";
	m_pTarget = target;
}

Arrival::Arrival( Arrival& orig) 
{
	m_name = "arrival";
	m_pTarget = orig.m_pTarget;
}

Arrival::~Arrival()
{
}

vec3 Arrival::calcDesiredVel( BehaviorController* actor)
{
	// TODO: add your code here to compute Vdesired
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	vec3 Xaxis, Yaxis, Zaxis;
	vec3 error = targetPos - actorPos;
	Yaxis = vec3(0.0, 1.0, 0.0);

	mat3 zRot = mat3();
	zRot.FromAxisAngle(Yaxis, actor->getOrientation()[_Y]);

	float speedCoef = actor->KArrival *  error.Length();

	Zaxis = zRot * vec3(0.0, 0.0, 1.0);

	Xaxis = Yaxis.Cross(Zaxis);

	double theta = atan2(Zaxis[0], Zaxis[2]);

	float errorXLength = error * Xaxis / Xaxis.Length();
	float errorZLength = error * Zaxis / Zaxis.Length();

	double transAngle = atan2(errorXLength, errorZLength);

	transAngle = theta + transAngle;

	mat3 velocityRot = mat3();
	velocityRot.FromAxisAngle(vec3(0.0, 1.0, 0.0), transAngle);

	Vdesired = speedCoef * velocityRot * vec3(0.0, 0.0, 1.0);

	actor->gVelKv = 10;
	actor->gOriKp = 256;
	actor->gOriKv = 32;

	return Vdesired;
}


// Departure behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// 1/(actor distance) from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Departure strength is in BehavioralController::KDeparture

Departure::Departure(AJoint* target) 
{
	m_name = "departure";
	m_pTarget = target;
}

Departure::Departure( Departure& orig) 
{
	m_name = "departure";
	m_pTarget = orig.m_pTarget;
}

Departure::~Departure()
{
}

vec3 Departure::calcDesiredVel( BehaviorController* actor)
{
	// TODO: add your code here to compute Vdesired
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	vec3 Xaxis, Yaxis, Zaxis;
	vec3 error = targetPos - actorPos;
	Yaxis = vec3(0.0, 1.0, 0.0);

	mat3 zRot = mat3();
	zRot.FromAxisAngle(Yaxis, actor->getOrientation()[_Y]);

	float speedCoef = actor->KDeparture / (error.Length() + 0.001);

	Zaxis = zRot * vec3(0.0, 0.0, 1.0);

	Xaxis = Yaxis.Cross(Zaxis);

	double theta = atan2(Zaxis[0], Zaxis[2]);

	float errorXLength = error * Xaxis / Xaxis.Length();
	float errorZLength = error * Zaxis / Zaxis.Length();

	double transAngle = atan2(errorXLength, errorZLength);

	transAngle = theta + transAngle + M_PI;

	mat3 velocityRot = mat3();
	velocityRot.FromAxisAngle(vec3(0.0, 1.0, 0.0), transAngle);

	Vdesired = speedCoef * velocityRot * vec3(0.0, 0.0, 1.0);

	actor->gVelKv = 10;
	actor->gOriKp = 256;
	actor->gOriKv = 32;

	return Vdesired;
}


// Avoid behavior
///////////////////////////////////////////////////////////////////////////////
//  For the given the actor, return a desired velocity in world coordinates
//  If an actor is near an obstacle, avoid adds a normal response velocity to the 
//  the desired velocity vector computed using arrival
//  Agent bounding sphere radius is in BehavioralController::radius
//  Avoidance parameters are  BehavioralController::TAvoid and BehavioralController::KAvoid

Avoid::Avoid(AJoint* target, vector<Obstacle>* obstacles) 
{
	m_name = "avoid";
	m_pTarget = target;
	mObstacles = obstacles;
}

Avoid::Avoid( Avoid& orig) 
{
	m_name = "avoid";
	m_pTarget = orig.m_pTarget;
	mObstacles = orig.mObstacles;
}

Avoid::~Avoid()
{
}

vec3 Avoid::calcDesiredVel( BehaviorController* actor)
{

	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	m_actorPos = actor->getPosition();
	m_actorVel = actor->getVelocity();

	//TODO: add your code here
	vec3 Varrival(0, 0, 0);
	// Step 1. compute initial value for Vdesired = Varrival so agent moves toward target
		// TODO: add your code here to compute Vdesired
	vec3 targetPos = m_pTarget->getGlobalTranslation();

	// TODO: add your code here to compute Vdesired
	vec3 Xaxis, Yaxis, Zaxis;
	vec3 error = targetPos - m_actorPos;
	Yaxis = vec3(0.0, 1.0, 0.0);

	mat3 zRot = mat3();
	zRot.FromAxisAngle(Yaxis, actor->getOrientation()[_Y]);

	float speedCoef = 0.6 * error.Length();

	Zaxis = zRot * vec3(0.0, 0.0, 1.0);

	Xaxis = Yaxis.Cross(Zaxis);

	double theta = atan2(Zaxis[0], Zaxis[2]);

	float errorXLength = error * Xaxis / Xaxis.Length();
	float errorZLength = error * Zaxis / Zaxis.Length();

	double transAngle = atan2(errorXLength, errorZLength);

	transAngle = theta + transAngle;

	mat3 velocityRot = mat3();
	velocityRot.FromAxisAngle(vec3(0.0, 1.0, 0.0), transAngle);

	Vdesired = speedCoef * velocityRot * vec3(0.0, 0.0, 1.0);

	actor->gVelKv = 10;
	actor->gOriKp = 256;
	actor->gOriKv = 32;

	vec3 Vavoid(0, 0, 0);
	//TODO: add your code here to compute Vavoid 



	// Step 2. compute Lb
	//TODO: add your code here
	//double tAvoid = 20;
	vec3 lB = actor->TAvoid * m_actorVel ;


	// Step 3. find closest obstacle 
	//TODO: add your code here
	double minDistance = DBL_MAX;
	int minIndex = 0;
	for (int i = 0; i < mObstacles->size(); i++) 
	{
		vec3 obstaclePos = mObstacles->at(i).m_Center.getGlobalTranslation();
		double distance = (obstaclePos - m_actorPos).Length();
		if (fabs(distance - minDistance) > DBL_EPSILON && distance < minDistance) 
		{
			m_obstaclePos = obstaclePos;
			minDistance = distance;
			minIndex = i;
		}
	}

	// Step 4. determine whether agent will collide with closest obstacle (only consider obstacles in front of agent)
	//TODO: add your code here
	vec3 dis = mObstacles->at(minIndex).m_Center.getGlobalTranslation() - m_actorPos;
	vec3 disZ = dis * Zaxis / Zaxis.Length();
	vec3 disX = dis * Xaxis / Xaxis.Length();
	double radiusSum = actor->gAgentRadius + mObstacles->at(minIndex).m_Radius;

	if (fabs(disZ.Length() - lB.Length()) > DBL_EPSILON&& disZ.Length() > lB.Length()) 
	{
		return Vdesired;
	}
	else if (fabs(disX.Length() - radiusSum) > DBL_EPSILON && disX.Length() > radiusSum) 
	{
		return Vdesired;
	}

	// Step 5.  if potential collision detected, compute Vavoid and set Vdesired = Varrival + Vavoid
	//TODO: add your code here

	Vavoid = (actor->KAvoid * (radiusSum - disX.Length()) / radiusSum) * (-disX).Normalize();
	Vdesired = Vdesired + Vavoid;
	return Vdesired;
}

void Avoid::display(BehaviorController* actor)
{
#ifdef DEBUG_BEHAVIORDISPLAY
	//  Draw Debug info
	vec3 angle = actor->getOrientation();
	vec3 vel = actor->getVelocity();
	vec3 dir = vec3(cos(angle[1]), 0, sin(angle[1]));
	vec3 probe = dir * (vel.Length() / BehaviorController::gMaxSpeed)*BehaviorController::TAvoid;

	glBegin(GL_LINES);
	glColor3f(0, 0, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_obstaclePos[0], m_obstaclePos[1], m_obstaclePos[2]);
	glColor3f(0, 1, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_actorPos[0] + probe[0], m_actorPos[1] + probe[1], m_actorPos[2] + probe[2]);
	glEnd();
#endif
}


// Wander Behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Wander returns a desired velocity vector whose direction changes at randomly from frame to frame
// Wander strength is in BehavioralController::KWander

Wander::Wander() 
{
	m_name = "wander";
	m_Wander = vec3(1.0, 0.0, 0.0);
}

Wander::Wander( Wander& orig) 
{
	m_name = "wander";
	m_Wander = orig.m_Wander;
}

Wander::~Wander()
{
}

vec3 Wander::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();

	// compute Vdesired = Vwander

	// Step. 1 find a random direction
	//TODO: add your code here
	float randDir = (rand() % (360 - 0 + 1)) + 0;
	vec3 randVec = vec3(cos(randDir * M_PI / 180.0), 0.0, sin(randDir * M_PI / 180.0)).Normalize();

	// Step2. scale it with a noise factor
	//TODO: add your code here
	vec3 rNoise = actor->KNoise * randVec;


	// Step3. change the current Vwander  to point to a random direction
	//TODO: add your code here
	m_Wander = actor->KWander * (m_Wander + rNoise).Normalize();

	// Step4. scale the new wander velocity vector and add it to the nominal velocity
	//TODO: add your code here
	Vdesired = 0.05 * actor->gMaxSpeed * vec3(1.0, 0.0, 1.0).Normalize() + m_Wander;

	return Vdesired;
}


// Alignment behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity vector in world coordinates
// Alignment returns the average velocity of all active agents in the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Alignment parameters are in BehavioralController::RNeighborhood and BehavioralController::KAlign


Alignment::Alignment(AJoint* target, vector<AActor>* agents) 
{
	m_name = "alignment";
	m_pAgentList = agents;
	m_pTarget = target;
}



Alignment::Alignment( Alignment& orig) 
{
	m_name = orig.m_name;
	m_pAgentList = orig.m_pAgentList;
	m_pTarget = orig.m_pTarget;

}

Alignment::~Alignment()
{
}

void Alignment::addAgent(AActor agent) 
{
	m_pAgentList->push_back(agent);
}

void Alignment::clearAgent() 
{
	m_pAgentList->clear();
}

vec3 Alignment::calcDesiredVel(BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getGlobalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_pAgentList;
	

	// compute Vdesired 
	
	// Step 1. compute value of Vdesired for fist agent (i.e. m_AgentList[0]) using an arrival behavior so it moves towards the target
	 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader

	if (actor == leader) 
	{
		vec3 Xaxis, Yaxis, Zaxis;
		vec3 error = targetPos - actorPos;
		Yaxis = vec3(0.0, 1.0, 0.0);

		mat3 zRot = mat3();
		zRot.FromAxisAngle(Yaxis, actor->getOrientation()[_Y]);

		float speedCoef = actor->KArrival * error.Length();

		Zaxis = zRot * vec3(0.0, 0.0, 1.0);

		Xaxis = Yaxis.Cross(Zaxis);

		double theta = atan2(Zaxis[0], Zaxis[2]);

		float errorXLength = error * Xaxis / Xaxis.Length();
		float errorZLength = error * Zaxis / Zaxis.Length();

		double transAngle = atan2(errorXLength, errorZLength);

		transAngle = theta + transAngle;

		mat3 velocityRot = mat3();
		velocityRot.FromAxisAngle(vec3(0.0, 1.0, 0.0), transAngle);

		Vdesired = speedCoef * velocityRot * vec3(0.0, 0.0, 1.0);

		actor->gVelKv = 10;
		actor->gOriKp = 256;
		actor->gOriKv = 32;

		return Vdesired;
	}
	// TODO: add your code here to compute Vdesired
	

	// Step 2. if not first agent compute Valign as usual
	//TODO: add your code here
	vec3 velocity = vec3();
	int count = 0;
	vec3 dis = actor->getPosition() - leader->getPosition();
	if (fabs(dis.Length() - actor->gKNeighborhood) > DBL_EPSILON && dis.Length() < actor->gKNeighborhood)
	{
		for (int i = 0; i < agentList.size(); i++)
		{
			BehaviorController* curAgent = agentList[i].getBehaviorController();
			velocity += curAgent->getVelocity();
			count++;
		}
		velocity = velocity / count;
		return velocity;
	}
	else 
	{
		return vec3(0.0, 0.0, 0.0);
	}
	
	
	
}

// Separation behavior
///////////////////////////////////////////////////////////////////////////////
// For the given te actor, return a desired velocity vector in world coordinates
// Separation tries to maintain a constant distance between all agents
// within the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Separation settings are in BehavioralController::RNeighborhood and BehavioralController::KSeperate

 

Separation::Separation( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "separation";
	m_AgentList = agents;
	m_pTarget = target;
}

Separation::~Separation()
{
}

Separation::Separation( Separation& orig) 
{
	m_name = "separation";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

vec3 Separation::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vseparate
	// TODO: add your code here to compute Vdesired 
	BehaviorController* leader = agentList[0].getBehaviorController();
	vec3 velocity = vec3();
	int count = 0;

	for (int i = 0; i < agentList.size(); i++)
	{
		BehaviorController* curAgent = agentList[i].getBehaviorController();
		vec3 dis = actor->getPosition() - curAgent->getPosition();
		if (fabs(dis.Length() - actor->gKNeighborhood) > DBL_EPSILON && dis.Length() < actor->gKNeighborhood) 
		{
			velocity += actor->KSeparation * dis / pow((dis.Length() + 0.001), 2);
			count++;
		}
	}

	if (count == 0) 
	{
		return Vdesired;
	}

	velocity = velocity / count;

	Vdesired = velocity;

	if (Vdesired.Length() < 5.0)
		Vdesired = 0.0;
	
	return Vdesired;
}


// Cohesion behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// Cohesion moves actors towards the center of the group of agents in the neighborhood
//  agents[i] gives the pointer to the ith agent in the environment
//  Cohesion parameters are in BehavioralController::RNeighborhood and BehavioralController::KCohesion


Cohesion::Cohesion( vector<AActor>* agents) 
{
	m_name = "cohesion";
	m_AgentList = agents;
}

Cohesion::Cohesion( Cohesion& orig) 
{
	m_name = "cohesion";
	m_AgentList = orig.m_AgentList;
}

Cohesion::~Cohesion()
{
}

vec3 Cohesion::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vcohesion
	// TODO: add your code here 
	BehaviorController* leader = agentList[0].getBehaviorController();
	vec3 velocity = vec3();
	int count = 0;
	vec3 centerPos = vec3();
	for (int i = 0; i < agentList.size(); i++)
	{
		BehaviorController* curAgent = agentList[i].getBehaviorController();
		vec3 dis = actor->getPosition() - curAgent->getPosition();
		if (fabs(dis.Length() - actor->gKNeighborhood) > DBL_EPSILON&& dis.Length() < actor->gKNeighborhood)
		{
			centerPos += curAgent->getPosition();
			count++;
		}
	}

	

	centerPos = centerPos / count;

	Vdesired = actor->KCohesion * (centerPos - actor->getPosition());

	if (count == 0)
	{
		return Vdesired;
	}

	return Vdesired;
}

// Flocking behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector  in world coordinates
// Flocking combines separation, cohesion, and alignment behaviors
//  Utilize the Separation, Cohesion and Alignment behaviors to determine the desired velocity vector


Flocking::Flocking( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "flocking";
	m_AgentList = agents;
	m_pTarget = target;
}

Flocking::Flocking( Flocking& orig) 
{
	m_name = "flocking";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Flocking::~Flocking()
{
}

vec3 Flocking::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	vec3 targetPos = m_pTarget->getGlobalTranslation();

	// compute Vdesired = Vflocking
	// TODO: add your code here 
	// 1.Alignment
	vec3 alignMentSpeed = vec3();
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader

	if (actor == leader)
	{
		vec3 Xaxis, Yaxis, Zaxis;
		vec3 error = targetPos - actorPos;
		Yaxis = vec3(0.0, 1.0, 0.0);

		mat3 zRot = mat3();
		zRot.FromAxisAngle(Yaxis, actor->getOrientation()[_Y]);

		float speedCoef = actor->KArrival * error.Length();

		Zaxis = zRot * vec3(0.0, 0.0, 1.0);

		Xaxis = Yaxis.Cross(Zaxis);

		double theta = atan2(Zaxis[0], Zaxis[2]);

		float errorXLength = error * Xaxis / Xaxis.Length();
		float errorZLength = error * Zaxis / Zaxis.Length();

		double transAngle = atan2(errorXLength, errorZLength);

		transAngle = theta + transAngle;

		mat3 velocityRot = mat3();
		velocityRot.FromAxisAngle(vec3(0.0, 1.0, 0.0), transAngle);

		alignMentSpeed = speedCoef * velocityRot * vec3(0.0, 0.0, 1.0);

		actor->gVelKv = 10;
		actor->gOriKp = 256;
		actor->gOriKv = 32;

	}
	else 
	{
		vec3 velocity = vec3();
		int count = 0;
		vec3 dis = actor->getPosition() - leader->getPosition();
		if (fabs(dis.Length() - actor->gKNeighborhood) > DBL_EPSILON&& dis.Length() < actor->gKNeighborhood)
		{
			for (int i = 0; i < agentList.size(); i++)
			{
				BehaviorController* curAgent = agentList[i].getBehaviorController();
				velocity += curAgent->getVelocity();
				count++;
			}
			velocity = velocity / count;
			alignMentSpeed = velocity;
		}
		else
		{
			alignMentSpeed = vec3(0.0, 0.0, 0.0);
		}
	}

	// 2.Separation
	vec3 separationSpeed = vec3();
	int count = 0;

	for (int i = 0; i < agentList.size(); i++)
	{
		BehaviorController* curAgent = agentList[i].getBehaviorController();
		vec3 dis = actor->getPosition() - curAgent->getPosition();
		if (fabs(dis.Length() - actor->gKNeighborhood) > DBL_EPSILON&& dis.Length() < actor->gKNeighborhood)
		{
			separationSpeed += actor->KSeparation * dis / pow((dis.Length() + 0.001), 2);
			count++;
		}
	}

	if (count == 0)
	{
		separationSpeed = vec3(0.0, 0.0, 0.0);
	}

	separationSpeed = separationSpeed / count;


	if (separationSpeed.Length() < 5.0)
		separationSpeed = 0.0;

	
	// 3.Cohesion
	vec3 cohesionSpeed = vec3();
	count = 0;
	vec3 centerPos = vec3();
	for (int i = 0; i < agentList.size(); i++)
	{
		BehaviorController* curAgent = agentList[i].getBehaviorController();
		vec3 dis = actor->getPosition() - curAgent->getPosition();
		if (fabs(dis.Length() - actor->gKNeighborhood) > DBL_EPSILON&& dis.Length() < actor->gKNeighborhood)
		{
			centerPos += curAgent->getPosition();
			count++;
		}
	}



	centerPos = centerPos / count;

	cohesionSpeed = actor->KCohesion * (centerPos - actor->getPosition());

	if (count == 0)
	{
		cohesionSpeed == vec3(0.0, 0.0, 0.0);
	}

	Vdesired = 2.0 * alignMentSpeed + 20 * separationSpeed + 3.0 * cohesionSpeed;

	return Vdesired;
}

//	Leader behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// If the agent is the leader, move towards the target; otherwise, 
// follow the leader at a set distance behind the leader without getting to close together
//  Utilize Separation and Arrival behaviors to determine the desired velocity vector
//  You need to find the leader, who is always agents[0]

Leader::Leader( AJoint* target, vector<AActor>* agents) 
{
	m_name = "leader";
	m_AgentList = agents;
	m_pTarget = target;
}

Leader::Leader( Leader& orig) 
{
	m_name = "leader";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Leader::~Leader()
{
}

vec3 Leader::calcDesiredVel( BehaviorController* actor)
{
	
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

	// TODO: compute Vdesired  = Vleader
	// followers should stay directly behind leader at a distance of -200 along the local z-axis

	float CSeparation = 20.0;  float CArrival = 0.6;

	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	mat3 Rmat = leader->getGuide().getLocalRotation();  // is rotattion matrix of lead agent
	vec3 targetPos = vec3();
	if (actor == leader) 
	{
		targetPos = actor->getTarget()->getGlobalTranslation();
	}
	else 
	{
		targetPos = actor->getTarget()->getGlobalTranslation() - Rmat * vec3(0.0, 0.0, 500.0);
	}

	vec3 arrivalSpeed = vec3();
	// 1. Arrival
	vec3 Xaxis, Yaxis, Zaxis;
	vec3 error = targetPos - actorPos;
	Yaxis = vec3(0.0, 1.0, 0.0);

	mat3 zRot = mat3();
	zRot.FromAxisAngle(Yaxis, actor->getOrientation()[_Y]);

	float speedCoef = actor->KArrival * error.Length();

	Zaxis = zRot * vec3(0.0, 0.0, 1.0);

	Xaxis = Yaxis.Cross(Zaxis);

	double theta = atan2(Zaxis[0], Zaxis[2]);

	float errorXLength = error * Xaxis / Xaxis.Length();
	float errorZLength = error * Zaxis / Zaxis.Length();

	double transAngle = atan2(errorXLength, errorZLength);

	transAngle = theta + transAngle;

	mat3 velocityRot = mat3();
	velocityRot.FromAxisAngle(vec3(0.0, 1.0, 0.0), transAngle);

	arrivalSpeed = speedCoef * velocityRot * vec3(0.0, 0.0, 1.0);

	actor->gVelKv = 10;
	actor->gOriKp = 256;
	actor->gOriKv = 32;

	// 2.Separation
	vec3 separationSpeed = vec3();
	int count = 0;

	for (int i = 0; i < agentList.size(); i++)
	{
		BehaviorController* curAgent = agentList[i].getBehaviorController();
		vec3 dis = actor->getPosition() - curAgent->getPosition();
		if (fabs(dis.Length() - actor->gKNeighborhood) > DBL_EPSILON && dis.Length() < actor->gKNeighborhood)
		{
			separationSpeed += actor->KSeparation * dis / pow((dis.Length() + 0.001), 2);
			count++;
		}
	}

	if (count == 0)
	{
		separationSpeed = vec3(0.0, 0.0, 0.0);
	}

	separationSpeed = separationSpeed / count;


	if (separationSpeed.Length() < 5.0)
		separationSpeed = 0.0;
	
	if (leader == actor) 
	{
		Vdesired = arrivalSpeed;
	}
	else
	{
		Vdesired = CArrival * arrivalSpeed + CSeparation * separationSpeed;
	}
	
	return Vdesired;
}

///////////////////////////////////////////////////////////////////////////////

