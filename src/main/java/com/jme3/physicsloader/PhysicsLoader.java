package com.jme3.physicsloader;

import com.jme3.scene.Spatial;

public interface PhysicsLoader<PHYSICS_OBJECT,PHYSICS_JOINT>{
	public PHYSICS_OBJECT load(PhysicsLoaderSettings settings,Spatial spatial,PhysicsData data);
	public PHYSICS_JOINT applyConstraint(PhysicsLoaderSettings settings,PHYSICS_OBJECT a,PHYSICS_OBJECT b,ConstraintData data);
}
