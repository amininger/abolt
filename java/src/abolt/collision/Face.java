package abolt.collision;

import april.jmat.LinAlg;
import april.vis.VisVertexData;

/**
 * Face is a support class for Shapes, it defines a face of a shape with 
 * a clockwise winding of vertices on that shape that form a convex face.
 * The winding order defines the normal direction (Right Handed), which determines the positive
 * and negative sides of the face's plane (positive being the same direction as the normal).
 */
public class Face {
	public enum RayType {
		ALLPOS, ALLNEG, POS2NEG, NEG2POS
	}
	public int[] indices;	// the index into the shape's vertices defining the face (clockwise ordering)
	public double[] normal; // The face's normal vector, determines the positive/negative sides of the plane
	public double[] point;  // A point on the face's plane
	
	public Face(double[][] vertices, int[] indices){
		this.indices = indices;
		if(indices.length < 3){
			System.err.println("Face Constructor: A face must have at least 3 indices");
			return;
		}
		for(int i = 0; i < indices.length; i++){
			if(indices[i] >= vertices.length){
				System.err.println("Face Constructor: index out of bounds for vertex array");
				return;
			}
		}
		recalcFace(vertices);
	}
	
	/**
	 * @param vertices - the vertices of the shape this face is associated with
	 * Needed to recalculate the normal and point on the face if the shape was transformed
	 */
	public void recalcFace(double[][] vertices){
		double[] edge1_2 = LinAlg.subtract(vertices[indices[2]], vertices[indices[1]]);
		double[] edge1_0 = LinAlg.subtract(vertices[indices[0]], vertices[indices[1]]);
		// Assumes a clockwise winding
		normal = LinAlg.crossProduct(edge1_2, edge1_0);
		if(LinAlg.normF(normal) == 0){
			System.err.println("Face Recalculation: Face has 0 area");
			return;
		}
		LinAlg.normalizeEquals(normal);
		point = vertices[indices[0]];
	}
	
	/**
	 * @param origin - origin of the ray
	 * @param dir - direction of the ray
	 * @return - the type of the given ray in relation to the plane
	 * @ALLPOS - the ray starts and continues on the positive side, never crossing the plane
	 * @ALLNEG - the ray starts and continues on the negative side, never crossing the plane
	 * @POS2NEG - the ray starts on the positive side and crosses the plane
	 * @NEG2POS - the ray starts on the negative side and crosses the plane
	 */
	public RayType getRayType(double[] origin, double[] dir){
		// True if the origin is on the positive side of the face
		boolean startPos = LinAlg.dotProduct(normal, LinAlg.subtract(origin, point)) > 0;
		
		// True if the normal is in the same direction as the ray's direction (pos dot product)
		boolean sameDir = LinAlg.dotProduct(normal, dir) > 0;
		
		if(startPos && sameDir){
			// startPos && sameDir
			return RayType.ALLPOS;
		} else if(startPos){
			// startPOS && diffDir
			return RayType.POS2NEG;
		} else if(sameDir){
			// startNEG && sameDir
			return RayType.NEG2POS;
		} else {
			// startNEG && diffDir
			return RayType.ALLNEG;
		}
	}
	
	/**
	 * @param origin - origin of the ray
	 * @param dir - direction of the ray
	 * @return - the distance from the origin along the ray to the intersection point of the face's plane
	 */
	public double rayDistToFacePlane(double[] origin, double[] dir){
		// plane intersection point is where:
		// normal [dot] (origin + s * dir - point) = 0, solve for s
		double denom = LinAlg.dotProduct(normal, dir);
		if(denom == 0){
			return Double.MAX_VALUE;
		}
		double num = LinAlg.dotProduct(normal, LinAlg.subtract(point, origin));
		return num / denom;
	}
	
	/**
	 * @param vertices - an array of points that make up the vertices of the shape which the Face indexes into
	 * @param origin - the origin of the ray
	 * @param dir - the direction of the ray
	 * @return - the distance from the origin along the ray to the intersection point of the face (inside the vertices), or
	 * MAX_VALUE if there is no intersection. The intersection point is calculated as
	 * the origin + distance * dir.
	 */
	public double rayDistToIntersection(double[][] vertices, double[] origin, double[] dir){
		double distToPlane = rayDistToFacePlane(origin, dir);
		if(distToPlane == Double.MAX_VALUE){
			return distToPlane;
		}
		double[] intPoint = LinAlg.add(origin, LinAlg.scale(dir, distToPlane));
		for(int i = 0; i < indices.length; i++){
			// Each edge of the face (going clockwise), start is labeled v0 and the end is labeled v1
			double[] v0 = vertices[indices[i]];
			double[] v1 = vertices[indices[(i+1)%indices.length]];
			// We assume that the face is convex. Therefore if the cross product of the edge vector (v1 - v0) and
			// the vector to the intersection point (intPoint - v0) is in the opposite direction as the normal
			// then the intersection point lies outside of the convex face and a negative collision is returned
			double[] cp = LinAlg.crossProduct(LinAlg.subtract(v1, v0), LinAlg.subtract(intPoint, v0));
			if(LinAlg.dotProduct(cp, normal) < 0){
				return Double.MAX_VALUE;
			}
		}
		return distToPlane;
	}
	
	public int fillVertexData(double[][] vertices, float[] vertexBuffer, int index){
		for(int i = 1; i < indices.length - 1; i++){
			vertexBuffer[index++] = (float)vertices[indices[0]][0];
			vertexBuffer[index++] = (float)vertices[indices[0]][1];
			vertexBuffer[index++] = (float)vertices[indices[0]][2];
			vertexBuffer[index++] = (float)vertices[indices[i]][0];
			vertexBuffer[index++] = (float)vertices[indices[i]][1];
			vertexBuffer[index++] = (float)vertices[indices[i]][2];
			vertexBuffer[index++] = (float)vertices[indices[i+1]][0];
			vertexBuffer[index++] = (float)vertices[indices[i+1]][1];
			vertexBuffer[index++] = (float)vertices[indices[i+1]][2];
		}
		return index;
	}
	
	public int fillNormalData(float[] normalBuffer, int index){
		for(int i = 0; i < 3 * (indices.length - 2); i++){
			for(int j = 0; j < 3; j++){
				normalBuffer[index++] = (float)normal[j];
			}
		}
		return index;
	}
}
