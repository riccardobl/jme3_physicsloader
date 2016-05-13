package com.jme3.physicsloader.rigidbody;

import com.jme3.math.Vector3f;
import com.jme3.physicsloader.PhysicsData;
import com.jme3.physicsloader.PhysicsShape;

public class RigidBody implements PhysicsData{

	public RigidBodyType type=RigidBodyType.NONE;
	public PhysicsShape shape=PhysicsShape.MESH;
	public float mass=1.f,friction=1.f,
			angularDamping=0f, 
			linearDamping=0f,
			margin=0f,restitution=0f;
	public Vector3f angularFactor=new Vector3f(1.0f,1.0f,1.0f),
			linearFactor=new Vector3f(1.0f,1.0f,1.0f);
	public boolean isKinematic=false;
	public int collisionMask=1,collisionGroup=1;

}
