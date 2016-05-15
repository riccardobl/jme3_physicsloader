package com.jme3.physicsloader.constraint;

import com.jme3.math.Vector3f;
import com.jme3.physicsloader.ConstraintData;

public class GenericConstraint implements ConstraintData{
	public Vector3f pivotA,pivotB,upperLinearLimit,lowerLinearLimit,upperAngularLimit,lowerAngularLimit;
}
