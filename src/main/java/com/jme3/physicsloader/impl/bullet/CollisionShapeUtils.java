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
import com.jme3.math.Vector3f;
import com.jme3.physicsloader.PhysicsLoaderSettings;
import com.jme3.physicsloader.PhysicsShape;
import com.jme3.physicsloader.utils.Helpers;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.SceneGraphVisitor;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer.Type;

public class CollisionShapeUtils{
	public static CollisionShape buildCollisionShape(final PhysicsLoaderSettings settings, final Spatial spatial,PhysicsShape pshape,final boolean dynamic,final boolean useCompoundCapsule,Logger logger){
		final CollisionShapeCacheEntry cache_entry=new CollisionShapeCacheEntry();
		cache_entry.savable=spatial;
		cache_entry.dynamic=dynamic;
		cache_entry.useCompoundCapsule=useCompoundCapsule;
		cache_entry.type=pshape.ordinal();
		
		if(settings.getCacher()!=null&&pshape!=PhysicsShape.MESH){
			CollisionShape out=settings.getCacher().load(cache_entry);
			if(out!=null)return out;
		}
		
		CollisionShape collisionShape=null;
	
		switch(pshape){
			case MESH:
				Object vhacd_factory=settings.getVHACDFactory();
				if(vhacd_factory!=null&&vhacd_factory instanceof Boolean) vhacd_factory=((boolean)vhacd_factory)==true?new com.jme3.bullet.vhacd.VHACDCollisionShapeFactory():null;

				final Object vhacd_factoryf=vhacd_factory;
				final CompoundCollisionShape csh=new CompoundCollisionShape();
				spatial.depthFirstTraversal(new SceneGraphVisitor(){
					@Override
					public void visit(Spatial s) {
						if(s instanceof Geometry){
							Geometry g=(Geometry)s;
							Mesh mesh=g.getMesh();
							CollisionShape shape=null;
							if(!dynamic){
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
				collisionShape=useCompoundCapsule?buildCompoundCapsule(r,xtendcapsule.y*2f):new CapsuleCollisionShape(r,xtendcapsule.y*2f-(r*2f));
				break;

			case CYLINDER:
				BoundingBox cybox=Helpers.getBoundingBox(spatial);
				Vector3f xtendcylinder=cybox.getExtent(null);
				collisionShape=new CylinderCollisionShape(xtendcylinder,PhysicsSpace.AXIS_Y);
				break;

			case CONE:
				BoundingBox cobox=Helpers.getBoundingBox(spatial);
				Vector3f xtendcone=cobox.getExtent(null);
				collisionShape=new ConeCollisionShape((xtendcone.x>xtendcone.z?xtendcone.x:xtendcone.z),xtendcone.y*2f,PhysicsSpace.AXIS_Y);
				break;
			default:
				// Should never happen.
				logger.log(Level.WARNING,"{0} unsupported",pshape);
		}
		
		if(settings.getCacher()!=null&&pshape!=PhysicsShape.MESH){
				settings.getCacher().store(cache_entry,collisionShape);
		}
		return collisionShape;

	}
	
	public static CollisionShape buildCompoundCapsule(float radius, float height) {
		float cylinder_height=height-(2.0f*radius);
		CylinderCollisionShape cylinder=new CylinderCollisionShape(new Vector3f(radius,cylinder_height/2f,radius)/*NB constructor want half extents*/,1);
		SphereCollisionShape sphere=new SphereCollisionShape(radius);
		CompoundCollisionShape compoundCollisionShape=new CompoundCollisionShape();
		compoundCollisionShape.addChildShape(sphere,new Vector3f(0,-cylinder_height/2f,0));
		compoundCollisionShape.addChildShape(cylinder,new Vector3f(0,0,0));
		compoundCollisionShape.addChildShape(sphere,new Vector3f(0,+cylinder_height/2f,0));
		return compoundCollisionShape;
	}
}
