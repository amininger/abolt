package abolt.collision;

public class ShapeFactory {
	/**
	 * @param vertices2d - the array of vertices in the x-y plane defining the polygon's face. 
	 * These should be convex and have a clockwise ordering
	 * @param height - the height of the desired shape
	 * @return - returns a flat polygon with the given vertices defining the flat face and
	 * then extruded vertically to give it volume
	 */
	public static Shape constructFlatPolygon(double[][] vertices2d, double height){
		int n = vertices2d.length;
		double[][] vertices = new double[2*n][3];
		for(int i = 0; i < vertices2d.length; i++){
			vertices[i] = new double[]{vertices2d[i][0], vertices2d[i][1], -height/2};
			vertices[n+i] = new double[]{vertices2d[i][0], vertices2d[i][1], height/2};
		}
		Face[] faces = new Face[n + 2];
		// Side faces
		for(int i = 0; i < n; i++){
			int[] face = new int[4];
			face[0] = i;
			face[1] = (i+1) % n;
			face[2] = (i+1) % n + n;
			face[3] = i + n;
			faces[i] = new Face(vertices, face);
		}
		
		// Top and bottom faces
		int[] topFace = new int[n];
		int[] botFace = new int[n];
		for(int i = 0; i < n; i++){
			topFace[i] = n + i;
			botFace[i] = n - 1 - i;
		}
		faces[n + 0] = new Face(vertices, topFace);
		faces[n + 1] = new Face(vertices, botFace);
		
		return new ConvexShape(vertices, faces);
	}
	
	/**
	 * @param numSides - number of sides for the polygon (>= 3)
	 * @param radius - the radius of the polygon face
	 * @param height - the height of the shape (>= 0)
	 * @return - A regular polygon of the given height, size, and number of sides
	 */
	public static Shape constructRegularFlatPolygon(int numSides, double radius, double height){
		assert (numSides > 2);
		double[][] vertices2d = new double[numSides][2];
		double dTheta = 2 * Math.PI / numSides;
		int i = 0;
		for(double theta = 0; theta < Math.PI * 2 - .0001f; theta += dTheta, i++){
			vertices2d[i][0] = Math.cos(theta) * radius;
			vertices2d[i][1] = Math.sin(theta) * radius;
		}
		return constructFlatPolygon(vertices2d, height);
	}

}
