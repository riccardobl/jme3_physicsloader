package com.jme3.physicsloader.impl;

import com.jme3.asset.ModelKey;
import com.jme3.physicsloader.PhysicsLoader;
import com.jme3.physicsloader.PhysicsLoaderSettings;
import com.jme3.physicsloader.impl.bullet.BulletPhysicsLoader;


public class PhysicsLoaderModelKey extends ModelKey implements PhysicsLoaderSettings {
	protected  PhysicsLoader<?> phyLoader;
	protected Object vhacdFactory;
	protected boolean	enhancedrbs=false;

	public PhysicsLoaderModelKey(){
		super();
	}
	
	public PhysicsLoaderModelKey(String path){
		super(path);
	}
	
	@Override
	public PhysicsLoaderModelKey usePhysics(boolean v){
		try{
			if(v)return usePhysics(new BulletPhysicsLoader());
			else return usePhysics(null);
		}catch(Throwable e){}
		return this;
	}

	@Override
	public Object getVHACDFactory(){
		return vhacdFactory;
	}
	
	@Override
	public PhysicsLoaderModelKey useVHACD(Object factory){
		if(factory ==null){
			vhacdFactory=null;
			return this;
		}
		try{
			if(factory instanceof com.jme3.bullet.vhacd.VHACDCollisionShapeFactory||factory instanceof Boolean){
				vhacdFactory=factory;
			}
		}catch(Throwable e){}
		return this;
	}
	
	@Override
	public PhysicsLoaderModelKey usePhysics(PhysicsLoader<?> phyLoader){
		this.phyLoader= phyLoader;
		return this;
	}
	
	@Override
	public PhysicsLoader<?> getPhysicsLoader(){
		return phyLoader;
	}

	@Override
	public PhysicsLoaderSettings useEnhancedRigidbodies(boolean v) {
		enhancedrbs=v;
		return this;
	}

	@Override
	public boolean useEnhancedRigidbodies() {
		return enhancedrbs;
	}
}
