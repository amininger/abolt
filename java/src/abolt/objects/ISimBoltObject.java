package abolt.objects;

import java.awt.Color;

import abolt.collision.Shape;
import april.sim.SimObject;

public interface ISimBoltObject extends SimObject
{	
	Color getColor();
	Shape getAboltShape();
	double[][] getPose();
	int getID();
}

