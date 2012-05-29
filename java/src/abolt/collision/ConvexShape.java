package abolt.collision;

import java.util.ArrayList;

import april.jmat.LinAlg;
import april.vis.VisVertexData;
import abolt.collision.Face.RayType;

public class ConvexShape implements Shape{
	public double[][] vertices;
	public Face[] faces;
	public double radius;
	float[] vertexBuffer;
	float[] normalBuffer;
	
	public ConvexShape(double[][] vertices, Face[] faces){
		this.vertices = vertices;
		this.faces = faces;
		radius = 0;
		for(int i = 0; i < vertices.length; i++){
			double mag = LinAlg.magnitude(vertices[i]);
			if(mag > radius){
				radius = mag;
			}
		}
		
		int nVertices = 0;
		for(int i = 0; i < faces.length; i++){
			nVertices += 3 * (faces[i].indices.length - 2); 
		}
		vertexBuffer = new float[3*nVertices];
		normalBuffer = new float[3*nVertices];
	}

	@Override
	public double getBoundingRadius() {
		return radius;
	}
	
	public void transform(double[][] T){
		for(int i = 0; i < vertices.length; i++){
			vertices[i] = LinAlg.transform(T, vertices[i]);
		}
		for(int i = 0; i < faces.length; i++){
			faces[i].recalcFace(vertices);
		}
	}

	@Override
	public double collisionRay(double[] _pos, double[] _dir, double[][] T) {
		double pos[] = LinAlg.transformInverse(T, _pos);
        double dir[] = LinAlg.transformInverseRotateOnly(T, _dir);
        
        RayType[] rayTypes = new RayType[faces.length];
        for(int i = 0; i < faces.length; i++){
        	rayTypes[i] = faces[i].getRayType(pos, dir);
        	if(rayTypes[i] == RayType.ALLPOS){
        		// The ray will never hit that plane, so early exit
        		return Double.MAX_VALUE;
        	}
        }

        // Distance to entering the convex hull of the faces
        double enter = 0;
        // Distance to exiting the convex hull of the faces
        double exit = Double.MAX_VALUE;
        
        for(int i = 0; i < faces.length; i++){
        	if(rayTypes[i] == RayType.ALLNEG){
        		// The ray is always 'inside' this face, so it doesn't change enter or exit
        		continue;
        	}
        	double dist = faces[i].rayDistToFacePlane(pos, dir);
        	if(rayTypes[i] == RayType.POS2NEG){
        		// 
        		if(dist > enter){
        			enter = dist;
        		}
        	} else {
        		if(dist < exit){
        			exit = dist;
        		}
        	}
        	if(enter > exit){
        		// There is no point along the ray where it is inside the convex hull
        		return Double.MAX_VALUE;
        	}
        }

        return enter;
	}
	
	public VisVertexData getVertexData(){
		int index = 0;
		for(Face face : faces){
			index = face.fillVertexData(vertices, vertexBuffer, index);
		}
		return new VisVertexData(vertexBuffer, vertexBuffer.length/3, 3);
	}
	
	public VisVertexData getNormals(){
		int index = 0;
		for(Face face : faces){
			index = face.fillNormalData(normalBuffer, index);
		}
		return new VisVertexData(normalBuffer, normalBuffer.length/3, 3);
	}
}
