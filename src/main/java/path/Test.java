package path;

import java.util.Arrays;

public class Test {
	public static void main(String[] args) {
		PathGenerator p = new PathGenerator();

		Path path = p.generate(0.4, 0.01, new Path(10, 100, 10, 3,
		        Arrays.asList(
		        		new Waypoint(0, 350), 
		        		new Waypoint(100, 350),
		                new Waypoint(150, 300), 
		                new Waypoint(150, 200),
		                new Waypoint(200, 150), 
		                new Waypoint(300, 150))));
		
		System.out.println(path);
	}
}
