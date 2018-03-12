package com.jme3.physicsloader;

public interface PhysicsLoaderSettings{

	/**
	 * @description Enable physics with the default physics loader [ default = false ]
	 * @param v
	 * @return
	 */
	public PhysicsLoaderSettings usePhysics(boolean v);

	/**
	 * @description Enable physics with the given physics loader [ default = null ]
	 * @param phyProvider null means disabled.
	 * @return
	 */
	public PhysicsLoaderSettings usePhysics(PhysicsLoader<?,?> phyProvider);

	public PhysicsLoader<?,?> getPhysicsLoader();

	/**
	 * @description Use VHACD to load dynamic mesh accurate shapes. [ default = null ]
	 * @param factory can be either an instance of VHACDCollisionShapeFactory or a boolean. 
	 * When a boolean is passed, the default implementation with default settings is used.
	 * null means disabled.
	 * @return
	 */
	public PhysicsLoaderSettings useVHACD(Object factory);

		
	/**
	 * @description Use VHACD to simplify mesh accurate static collision shapes.
	 * @param v [default=false]
	 * @return
	 */
	public PhysicsLoaderSettings useVHACDForStaticCollisions(boolean v);

	public boolean useVHACDForStaticCollisions();
	
	public Object getVHACDFactory();

	/**	
	 * @description [ default = false ]
	 * @param v
	 * @return
	 */
	public PhysicsLoaderSettings useEnhancedRigidbodies(boolean v);

	public boolean useEnhancedRigidbodies();
	
	

	
	/**
	 * @description 
	 * @param phyProvider null means disabled.
	 * @return
	 */
	public PhysicsLoaderSettings useCacher(PhysicsCacher cacher);

	public PhysicsCacher getCacher();
}
