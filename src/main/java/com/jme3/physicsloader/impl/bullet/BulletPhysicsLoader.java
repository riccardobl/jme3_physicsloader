package com.jme3.physicsloader.impl.bullet;

import static com.jme3.physicsloader.impl.bullet.RigidBodyUtils.*;

import java.util.concurrent.Callable;
import java.util.logging.Logger;

import com.jme3.app.AppTask;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.control.PhysicsControl;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.physicsloader.ConstraintData;
import com.jme3.physicsloader.PhysicsData;
import com.jme3.physicsloader.PhysicsLoader;
import com.jme3.physicsloader.PhysicsLoaderSettings;
import com.jme3.physicsloader.rigidbody.RigidBody;
import com.jme3.physicsloader.rigidbody.RigidBodyType;
import com.jme3.scene.Spatial;

public class BulletPhysicsLoader implements PhysicsLoader<PhysicsControl,PhysicsJoint>{
	private static final Logger logger=Logger.getLogger(BulletPhysicsLoader.class.getName());

	protected boolean useCompoundCapsule=false;
	protected Runner runner;

	/**
	 * Enqueue operations to physics space, useful when using detached threading model.
	 */
	public BulletPhysicsLoader useRunner(Runner ps) {
		runner=ps;
		return this;
	}

	@Override
	public void attachToSpatial(final PhysicsControl obj, final Spatial spatial) {
		try{
			safeRun(new Callable<Boolean>(){
				@Override
				public Boolean call() throws Exception {
					spatial.addControl(obj);
					return true;
				}
			});
		}catch(Exception e){
			e.printStackTrace();
		}
	}

	public synchronized Object safeRun(final Callable<?> r) throws Exception {
		return safeRun(r,true);
	}
	
	public synchronized Object safeRun(final Callable<?> r,boolean wait) throws Exception {
		if(runner==null) return r.call();
		else{
			return runner.run(r,wait);
		}
	}

	/**
	 *   Use a compound shape instead of CapsuleCollisionShape. See https://hub.jmonkeyengine.org/t/btcapsuleshape-location-isnt-accurate-at-all/35752/15 for more info.
	 * @param v
	 * @return
	 */
	public BulletPhysicsLoader useCompoundCapsule(boolean v) {
		useCompoundCapsule=v;
		return this;
	}

	public boolean useCompoundCapsule() {
		return useCompoundCapsule;
	}

	@Override
	public PhysicsJoint loadConstraint(final PhysicsLoaderSettings settings, final Object a, final Object b, final ConstraintData ct) {
		if(a instanceof RigidBodyControl&&b instanceof RigidBodyControl){
			try{
				return (PhysicsJoint)safeRun(new Callable<PhysicsJoint>(){

					@Override
					public PhysicsJoint call() throws Exception {
						return applyRBConstraint(settings,(RigidBodyControl)a,(RigidBodyControl)b,ct,logger);

					}

				});
			}catch(Exception e){
				e.printStackTrace();
			}
		}
		return null;
	}

	@Override
	public PhysicsControl load(final PhysicsLoaderSettings settings, final Spatial spatial, final PhysicsData data) {
		if(data instanceof RigidBody){
			try{
				return (PhysicsControl)safeRun(new Callable<PhysicsControl>(){
					@Override
					public PhysicsControl call() throws Exception {
						RigidBody rb=(RigidBody)data;
						if(rb.type==RigidBodyType.GHOST) return loadGhost(settings,spatial,rb,useCompoundCapsule,logger);
						else return loadRB(settings,spatial,rb,useCompoundCapsule,logger);
					}
				});
			}catch(Exception e){
				e.printStackTrace();
			}
		}
		return null;
	}



}
