package com.jme3.physicsloader;

import com.jme3.math.Vector3f;

public class PhysicsData{
	public static enum PhysicsType{
		STATIC,
		DYNAMIC,
		NONE
	}
	public static enum PhysicsShape{
		MESH,
		SPHERE,
		HULL,
		BOX,
		CAPSULE,
		CYLINDER,
		CONE
	}
	public PhysicsType type=PhysicsType.NONE;
	public PhysicsShape shape=PhysicsShape.MESH;
	public float mass=1.f,friction=1.f,
			angularDamping=0f, 
			linearDamping=0f,
			margin=0f,restitution=0f;
	public Vector3f angularFactor=new Vector3f(1.0f,1.0f,1.0f),
			linearFactor=new Vector3f(1.0f,1.0f,1.0f);
	public boolean isGhost=false,isKinematic=false;
	public int collisionMask=1,collisionGroup=1;

}
