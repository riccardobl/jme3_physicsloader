package com.jme3.physicsloader.impl.bullet;

import java.util.ArrayList;
import java.util.Collection;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.math.Vector3f;
import com.jme3.physicsloader.PhysicsData;
import com.jme3.physicsloader.PhysicsLoader;
import com.jme3.physicsloader.PhysicsLoaderSettings;
import com.jme3.physicsloader.rigidbody.RigidBody;
import com.jme3.physicsloader.rigidbody.RigidBodyType;
import com.jme3.physicsloader.utils.Helpers;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.SceneGraphVisitor;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;

public class BulletPhysicsLoader implements PhysicsLoader<Control>{
	private static final Logger logger=Logger.getLogger(BulletPhysicsLoader.class.getName());
	
	@Override
	public Control load(PhysicsLoaderSettings settings, Spatial spatial, PhysicsData data) {
		if(data instanceof RigidBody)return loadRB(settings,spatial,(RigidBody)data);
		
		return null;
	}
	
	protected Control loadRB(final PhysicsLoaderSettings settings,final Spatial spatial,final RigidBody data){

		if(data.type==RigidBodyType.NONE)return null;
		CollisionShape collisionShape=null;

		switch(data.shape){
			case MESH:
				Object vhacd_factory=settings.getVHACDFactory();
				if(vhacd_factory!=null&&vhacd_factory instanceof Boolean)vhacd_factory=((boolean)vhacd_factory)==true?new com.jme3.bullet.vhacd.VHACDCollisionShapeFactory():null;
								
				final Object vhacd_factoryf=vhacd_factory;
				final CompoundCollisionShape csh=new CompoundCollisionShape();
				spatial.depthFirstTraversal(new SceneGraphVisitor(){
					@Override
					public void visit(Spatial s) {
						if(s instanceof Geometry){
							Geometry g=(Geometry)s;
							Mesh mesh=g.getMesh();
							CollisionShape shape=null;
							if(data.type==RigidBodyType.STATIC){
								shape=new MeshCollisionShape(mesh);
							}else{
								if(vhacd_factoryf!=null){				
									com.jme3.bullet.vhacd.VHACDCollisionShapeFactory f=(com.jme3.bullet.vhacd.VHACDCollisionShapeFactory)vhacd_factoryf;
									CompoundCollisionShape ccs=f.create(spatial);
									for(ChildCollisionShape c:ccs.getChildren()){
										c.shape.setScale(g.getWorldScale());
										csh.addChildShape(c.shape,g.getWorldTranslation().subtract(spatial.getWorldTranslation()));
									}				
								}else shape=new GImpactCollisionShape(mesh);
							}
							if(shape!=null){
								shape.setScale(g.getWorldScale());
								csh.addChildShape(shape,g.getWorldTranslation().subtract(spatial.getWorldTranslation()));
							}
						}
					}
				});
				collisionShape=csh;
				break;

			case SPHERE:
				Vector3f xtendsphere=Helpers.getBoundingBox(spatial).getExtent(null);
				float radius=xtendsphere.x;
				if(xtendsphere.y>radius) radius=xtendsphere.y;
				if(xtendsphere.z>radius) radius=xtendsphere.z;
				collisionShape=new SphereCollisionShape(radius);
				break;

			case HULL:
				final Collection<Float> points=new ArrayList<Float>();
				spatial.depthFirstTraversal(new SceneGraphVisitor(){
					@Override
					public void visit(Spatial s) {
						if(s instanceof Geometry){
							Geometry g=(Geometry)s;
							points.addAll(Helpers.getPoints(g.getMesh(),g.getWorldScale()));
						}
					}
				});
				float primitive_arr[]=new float[points.size()];
				int i=0;
				for(Float point:points)
					primitive_arr[i++]=(float)point;
				collisionShape=new HullCollisionShape(primitive_arr);
				break;

			case BOX:
				BoundingBox bbox=Helpers.getBoundingBox(spatial);
				collisionShape=new BoxCollisionShape(bbox.getExtent(null));
				break;

			case CAPSULE:
				BoundingBox cbox=Helpers.getBoundingBox(spatial);
				Vector3f xtendcapsule=cbox.getExtent(null);
				float r=(xtendcapsule.x>xtendcapsule.z?xtendcapsule.x:xtendcapsule.z);
				collisionShape=new CapsuleCollisionShape(r,xtendcapsule.y-r*2f);
				break;

			case CYLINDER:
				BoundingBox cybox=Helpers.getBoundingBox(spatial);
				Vector3f xtendcylinder=cybox.getExtent(null);
				collisionShape=new CylinderCollisionShape(xtendcylinder);
				break;

			case CONE:
				BoundingBox cobox=Helpers.getBoundingBox(spatial);
				Vector3f xtendcone=cobox.getExtent(null);
				collisionShape=new ConeCollisionShape((xtendcone.x>xtendcone.z?xtendcone.x:xtendcone.z),xtendcone.y,PhysicsSpace.AXIS_Y);
				break;
			default:
				// Should never happen.
				logger.log(Level.WARNING,"{0} unsupported",data.shape);
		}
		
		if(collisionShape==null) return null;
		collisionShape.setMargin(data.margin);
		if(data.type==RigidBodyType.GHOST){
			GhostControl ghost=new GhostControl(collisionShape);
			return ghost;
		}else{
			float mass=data.type==RigidBodyType.STATIC?0:data.mass;
			RigidBodyControl rigidbody=settings.useEnhancedRigidbodies()?new BulletEnhancedRigidBodyControl(collisionShape,mass):new RigidBodyControl(collisionShape,mass);
			rigidbody.setFriction(data.friction);
			rigidbody.setAngularDamping(data.angularDamping);
			rigidbody.setLinearDamping(data.linearDamping);
			rigidbody.setLinearFactor(data.linearFactor);
			rigidbody.setAngularFactor(data.angularFactor);
			rigidbody.setKinematic(data.isKinematic);
			rigidbody.setRestitution(data.restitution);
			rigidbody.setCollisionGroup(data.collisionGroup);
			rigidbody.setCollideWithGroups(data.collisionMask);
			return rigidbody;
		}
	}

	
}
