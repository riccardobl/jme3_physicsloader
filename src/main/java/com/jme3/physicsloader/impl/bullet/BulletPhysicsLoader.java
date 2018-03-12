package com.jme3.physicsloader.impl.bullet;

import static com.jme3.physicsloader.impl.bullet.RigidBodyUtils.*;

import java.util.logging.Logger;

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
	public PhysicsJoint loadConstraint(PhysicsLoaderSettings settings, Object a, Object b, ConstraintData ct) {
		if(a instanceof RigidBodyControl && b instanceof RigidBodyControl)return applyRBConstraint(settings,(RigidBodyControl)a,(RigidBodyControl)b,ct,logger);
		return null;
	}

	@Override
	public PhysicsControl load(PhysicsLoaderSettings settings, Spatial spatial, PhysicsData data) {
		if(data instanceof RigidBody) {
			RigidBody rb=(RigidBody)data;
			if(rb.type==RigidBodyType.GHOST) return loadGhost(settings,spatial,rb,useCompoundCapsule,logger);
			else return loadRB(settings,spatial,rb,useCompoundCapsule,logger);
		}
		return null;
	}

}
