package com.jme3.physicsloader.impl.bullet;

import java.util.concurrent.atomic.AtomicBoolean;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.physicsloader.EnhancedRigidBodyControl;

public class BulletEnhancedRigidBodyControl extends RigidBodyControl implements EnhancedRigidBodyControl{
	protected AtomicBoolean locationUpdatedFromPhysicsK=new AtomicBoolean(false),rotationUpdatedFromPhysicsK=new AtomicBoolean(false),updateTransformFromSpatial=new AtomicBoolean(false);

	public BulletEnhancedRigidBodyControl(float mass){
		super(mass);
	}

	public BulletEnhancedRigidBodyControl(CollisionShape shape){
		super(shape);
	}

	public BulletEnhancedRigidBodyControl(CollisionShape shape,float mass){
		super(shape,mass);
	}

	@Override
	public BulletEnhancedRigidBodyControl jmeClone() {
		BulletEnhancedRigidBodyControl control=new BulletEnhancedRigidBodyControl(collisionShape,mass);
		control.spatial=this.spatial;
		control.setAngularFactor(getAngularFactor());
		control.setAngularSleepingThreshold(getAngularSleepingThreshold());
		control.setCcdMotionThreshold(getCcdMotionThreshold());
		control.setCcdSweptSphereRadius(getCcdSweptSphereRadius());
		control.setCollideWithGroups(getCollideWithGroups());
		control.setCollisionGroup(getCollisionGroup());
		control.setDamping(getLinearDamping(),getAngularDamping());
		control.setFriction(getFriction());
		control.setGravity(getGravity());
		control.setKinematic(isKinematic());
		control.setKinematicSpatial(isKinematicSpatial());
		control.setLinearSleepingThreshold(getLinearSleepingThreshold());
		control.setPhysicsLocation(getPhysicsLocation(null));
		control.setPhysicsRotation(getPhysicsRotationMatrix(null));
		control.setRestitution(getRestitution());
		if(mass>0){
			control.setAngularVelocity(getAngularVelocity());
			control.setLinearVelocity(getLinearVelocity());
		}
		control.setApplyPhysicsLocal(isApplyPhysicsLocal());
		return control;
	}

	@Override
	public void setPhysicsSpace(PhysicsSpace space) {
		super.setPhysicsSpace(space);
		if(space!=null&&spatial!=null){
			updateTransformFromSpatial.set(true);
		}
	}

	private Vector3f getSpatialTranslation() {
		if(motionState.isApplyPhysicsLocal()){ return spatial.getLocalTranslation(); }
		return spatial.getWorldTranslation();
	}

	private Quaternion getSpatialRotation() {
		if(motionState.isApplyPhysicsLocal()){ return spatial.getLocalRotation(); }
		return spatial.getWorldRotation();
	}

	@Override
	public void setPhysicsLocation(Vector3f v) {
		super.setPhysicsLocation(v);
		if(isKinematic()){
			locationUpdatedFromPhysicsK.set(true);
		}
	}

	@Override
	public void setPhysicsRotation(Quaternion r) {
		super.setPhysicsRotation(r);
		if(isKinematic()){
			rotationUpdatedFromPhysicsK.set(true);
		}
	}

	@Override
	public void setPhysicsRotation(Matrix3f r) {
		super.setPhysicsRotation(r);

		if(isKinematic()){
			rotationUpdatedFromPhysicsK.set(true);
		}
	}

	@Override
	public void update(float tpf) {
		if(space==null) return;

		if(enabled&&spatial!=null){
			if(updateTransformFromSpatial.getAndSet(false)){
				setPhysicsLocation(getSpatialTranslation());
				setPhysicsRotation(getSpatialRotation());
			}

			if(isKinematic()&&kinematicSpatial){
				boolean lfp=locationUpdatedFromPhysicsK.getAndSet(false);
				boolean rfp=rotationUpdatedFromPhysicsK.getAndSet(false);
				if(!lfp) super.setPhysicsLocation(getSpatialTranslation());
				if(!rfp) super.setPhysicsRotation(getSpatialRotation());
				if(lfp||rfp) getMotionState().applyTransform(spatial);
			}else{
				getMotionState().applyTransform(spatial);
			}
		}
	}
}
