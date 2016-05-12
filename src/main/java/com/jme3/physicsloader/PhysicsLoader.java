package com.jme3.physicsloader;

import com.jme3.scene.Spatial;

public interface PhysicsLoader<R>{
	public R load(PhysicsLoaderSettings settings,Spatial spatial,PhysicsData data);
}
