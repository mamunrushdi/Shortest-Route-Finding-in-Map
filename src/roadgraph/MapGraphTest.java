package roadgraph;

import java.util.List;

import geography.GeographicPoint;

public class MapGraphTest {

	public static void main(String[] args) 
	{
	
		MapGraph mapBuilder = new MapGraph();
		
		GeographicPoint a = new GeographicPoint(0.0, 0.0);
		mapBuilder.addVertex(a);
		
		GeographicPoint b = new GeographicPoint(-1.0, 1.0);
		mapBuilder.addVertex(b);
		
		GeographicPoint c = new GeographicPoint(1.0, 2.0);
		mapBuilder.addVertex(c);
		
		GeographicPoint d = new GeographicPoint(0.0, 3.0);
		mapBuilder.addVertex(d);
		
		GeographicPoint e = new GeographicPoint(4.0 ,4.0);
		mapBuilder.addVertex(e);
		
		GeographicPoint f = new GeographicPoint(0.0, 4.0);
		mapBuilder.addVertex(f);
		
		
		mapBuilder.addEdge(a, b, "Zig", "edge", 157);
		mapBuilder.addEdge(b, c, "Zag", "edge", 248);
		mapBuilder.addEdge(c, d, "Left", "edge", 157);
		mapBuilder.addEdge(d, e, "Upper", "edge", 458);
		mapBuilder.addEdge(e, f, "Mid", "Resedentail", 444);
		mapBuilder.addEdge(a, e, "Right", "Bypass", 628);
		
		mapBuilder.addEdge(b, a, "Zig", "edge", 157);
		mapBuilder.addEdge(c, b, "Zag", "edge", 248);
		mapBuilder.addEdge(d, c, "Left", "edge", 157);
		mapBuilder.addEdge(e, d, "Upper", "edge", 458);
		mapBuilder.addEdge(f, e, "Mid", "Resedentail", 444);
		mapBuilder.addEdge(e, a, "Right", "Bypass", 628);
		
		
		//mapBuilder.printGraph();
//		
		//print map information
		System.out.println("Total location in the map: " + mapBuilder.getNumVertices());
		System.out.println("Total edges in the map: " + mapBuilder.getNumEdges());
		
		GeographicPoint start = new GeographicPoint(0.0, 0.0);
		GeographicPoint goal = new GeographicPoint(0.0, 0.4);
		
		System.out.println("---------------");
		List<GeographicPoint> pathDijkstra = mapBuilder.dijkstra(a, f);

		System.out.println("Path in Dijkstra:" + pathDijkstra);
		System.out.println("Total node visited by Dijkstra: " + mapBuilder.getDijkstraNodeNumber());
		
		System.out.println("---------------");
		
		List<GeographicPoint> pathAStart = mapBuilder.aStarSearch(a, f);
		System.out.println("AStar Path:" + pathAStart);
		System.out.println("Total node visited by AStar: " + mapBuilder.getAStarNodeNumber());
		
//		//add location to the map
//		GeographicPoint a = new GeographicPoint(1.0, 1.0);
//		mapBuilder.addVertex(a);
//		
//		GeographicPoint b = new GeographicPoint(4.0, 1.0);
//		mapBuilder.addVertex(b);
//		
//		GeographicPoint c = new GeographicPoint(4.0, 2.0);
//		mapBuilder.addVertex(c);
//		
//		GeographicPoint d = new GeographicPoint(7.0, 3.0);
//		mapBuilder.addVertex(d);
//		
//		GeographicPoint e = new GeographicPoint(4.0,0.0);
//		mapBuilder.addVertex(e);
//		
//		GeographicPoint f = new GeographicPoint(5.0, 1.0);
//		mapBuilder.addVertex(f);
//		
//		GeographicPoint g = new GeographicPoint(6.5, 0.0);
//		mapBuilder.addVertex(g);
//		
//		GeographicPoint h = new GeographicPoint(4.0, -1.0);
//		mapBuilder.addVertex(h);
//		
//		GeographicPoint i = new GeographicPoint(8.0, -1.0);
//		mapBuilder.addVertex(i);	
//		
//		
//		//add neighbor
//		mapBuilder.addEdge(a, b, "Magistral1", "Magistral", 330);
//		mapBuilder.addEdge(b, c, "Resedential1", "Resedential", 110);
//		mapBuilder.addEdge(b, d, "Magistral2", "Magistral", 400);
//		mapBuilder.addEdge(b, e, "Main1", "Main", 110);
//		mapBuilder.addEdge(b, f, "Main2", "Main", 111);
//		mapBuilder.addEdge(e, f, "Main3", "main", 157);
//		mapBuilder.addEdge(e, h, "Resedential2", "Resedential", 110);
//		mapBuilder.addEdge(f, g, "Local1", "Local", 200);
//		mapBuilder.addEdge(g, i, "Local2", "Local", 200);
//		mapBuilder.addEdge(h, i, "Resedential3", "Resedntial", 444);
//		mapBuilder.addEdge(d, i, "Resedential4", "Resedential", 454);
//		
//
//		mapBuilder.addEdge(b, a, "Magistral1", "Magistral", 330);
//		mapBuilder.addEdge(c, b, "Resedential1", "Resedential", 110);
//		mapBuilder.addEdge(d, b, "Magistral2", "Magistral", 400);
//		mapBuilder.addEdge(e, b, "Main1", "Main", 110);
//		mapBuilder.addEdge(f, b, "Main2", "Main", 111);
//		mapBuilder.addEdge(f, e, "Main3", "main", 157);
//		mapBuilder.addEdge(h, e, "Resedential2", "Resedential", 110);
//		mapBuilder.addEdge(g, f, "Local1", "Local", 200);
//		mapBuilder.addEdge(i, g, "Local2", "Local", 200);
//		mapBuilder.addEdge(i, h, "Resedential3", "Resedntial", 444);
//		mapBuilder.addEdge(i, d, "Resedential4", "Resedential", 454);
//		
		

		
	}

}
