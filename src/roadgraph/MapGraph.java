/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */

/**
 * This is an extension of the project.
 * In this extension route will be calculated based upon the type of road and time of the day.
 * 
 * If the comparison in the priority list is among "Bypass", "Avenue" and "Residential" road
 * then Bypass will get priority by 10% of its weight which means 10% weight will be decreased from the 
 * weight of the Bypass (this deduction occurs in the MapEdge node during the edge constructions)
 * 
 *  If it is Rush hours of day :
 *  						Rush hour in the morning: From 8 a.m. to 10:30 a.m.
 *  						Rush hour in the evening: From 5 p.m to 7:30 p,m.
 *  during these time, if the comparison in the priority list is between a residential street and avenue
 *  then then the residential street will get 20% of its weight means 20% weight from its total weight
 *   will be decreased from the
 *  Residential street's weight
 * 
 * If its in the mid night:
 * 							Mid night : From 10 P.M. to 7 A.M.
 * During this time in comparison between avenue and residential street will get priority by 20% of its weight
 * 
 */
package roadgraph;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
 
import geography.GeographicPoint;
import util.GraphLoader;  

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */

/**
 * 
 * @author mamun
 * 
 * As an extension of this program, now the program calculate the route according to the day time and street type.
 * 
 * In this map, we have three type of street:
 * 											1. Bypass
 * 											2. Avenue
 * 											3. Residential
 * 
 * If the street type is "Bypass" then its weight will be decreased by 10% of its weight. 
 * These calculation occurs in MapEdge construction.
 *
 *If the street is type of "Avenue" and the day time in in Rush hour then the route calculation
 *is calculated following:
 * 
 *  If it is in Rush hours of day :
 *  						Rush hour in the morning: From 8 a.m. to 10:30 a.m.
 *  						Rush hour in the evening: From 5 p.m to 7:30 p,m.
 *  during these time, if the comparison in the priority list is between a residential street and avenue
 *  then then the residential street will get 10% of its weight means 10% weight from its total weight
 *   will be decreased from the
 *  Residential street's weight
 *   
 * If its in the mid night:
 * 							Mid night : From 10 P.M. to 7 A.M.
 * During this time in comparison between avenue and residential street will get priority by 20% of its weight
 * 
 */
public class MapGraph 
{
	//represent the cityMap
	private ArrayList<MapNode> nodeList;
	
	private Map<GeographicPoint, ArrayList<MapEdge>> map;
	
	private int numNode;

	private int numEdges;
	
	private int dijkstraNodeNumber; //store the total number of node visited by Dijkstra algorithm
	private int aStarNodeNumber; //store the number of total node visited by aStar algorithm
	
	private int currentTime;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{

		map = new HashMap<>();
		nodeList =  new ArrayList<>();
		numNode = 0;
		numEdges = 0;
		dijkstraNodeNumber = 0;
		aStarNodeNumber = 0;
		
		currentTime = getCurrentTime();
	}
	
	public int getCurrentTime()
	{
		Calendar cal = Calendar.getInstance();
		return cal.get(Calendar.HOUR_OF_DAY);
	}
	//return the city number
	public int getNumVertices()
	{
		return numNode;
	}
	
	public Map<GeographicPoint, ArrayList<MapEdge>> getCityMap()
	{
		return map;
	}
	public ArrayList<MapNode> getCityNodeList()
	{
		return nodeList;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		Set<GeographicPoint> nodeSet = map.keySet();
		
		Set<GeographicPoint> gpSet = new HashSet<>();
		
		for(GeographicPoint v : nodeSet)
			gpSet.add(v);
		
		return gpSet;
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{  	 
		if(map.containsKey(location)) throw new IllegalArgumentException();
		
		map.put(location, new ArrayList<>()); //create a map for the city with an array
		
		numNode++; //update the city number by 1
		
		return true;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}	
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) throws IllegalArgumentException 
	{
		//add neighbor only if the given cities are already added to the map
		if(!map.containsKey(from) || !map.containsKey(to)) throw new IllegalArgumentException();
		//create an edge	
		MapEdge edge = new MapEdge(to, roadName, roadType, length);
		
		//get the neighbor list of the given city
		ArrayList<MapEdge> neighbor = map.get(from); 
		
		neighbor.add(edge);
		//update the map
		map.put(from, neighbor);
		
		//after each successfully adding edge, increase the edge number by 1
		numEdges++;
   }
	
	//override toString method
	public void printVertices()
	{
		Set<GeographicPoint> vertexSet = map.keySet();
		
		for(GeographicPoint m : vertexSet)
			System.out.println(m);
	}

	//print the graph for debugging purpose
	public void printGraph()
	{
		Set<GeographicPoint> keySet = map.keySet();
		for(GeographicPoint m : keySet)
		{
			System.out.println(m + " -> " + edgeDetails(map.get(m)));
		}
	}
	
	//print edge details
	private String edgeDetails(ArrayList<MapEdge> edgeList)
	{
		String ret = "";		
		for(MapEdge e : edgeList)		
			ret += "{" +e.getNeighborCityName()  +", " + e.getRoadName()+", " + e.getRoadType() +", "+ e.getLength()+"}, ";
		return ret;
	}
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) 
	{
		// Dummy variable for calling the search algorithms
 		if (start == null || goal == null) 
		{
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
 		
        Consumer<GeographicPoint> temp = (x) -> {};
        
        List<GeographicPoint> path = bfs(start, goal, temp);
 
        return path;
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{ 
		HashSet<GeographicPoint> visited = new HashSet<>();
		Queue<GeographicPoint> toExplore = new LinkedList<>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		
		toExplore.add(start);
		
		boolean found = false;
		
		while (!toExplore.isEmpty()) 
		{ 
			GeographicPoint curr = toExplore.remove();
			
			if (curr.equals(goal)) //if the goal location is found
			{
				found = true;
				break;
			}
			
			ArrayList<MapEdge> neighbors = map.get(curr); 
				
			Iterator<MapEdge> it = neighbors.iterator();
			while(it.hasNext())
			{
				MapEdge next = it.next();
				nodeSearched.accept(next.getLocation());
				
				if(!visited.contains(next.getLocation()))
				{
					visited.add(next.getLocation());
					parentMap.put(next.getLocation(), curr);
					toExplore.add(next.getLocation());
				}
			}
		}
		if (!found) 
		{
			System.out.println("No path exists");
			return null;
		}
		// reconstruct the path
		 return constructPath(start, goal, parentMap); 
	}
	
	//helper method to construct the path
	private static List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal, Map<GeographicPoint, GeographicPoint> parentMap) 
	{
		LinkedList<GeographicPoint> path = new LinkedList<>();
		GeographicPoint curr = goal;
		while (curr != start) 
		{
			path.addFirst(curr);
			curr = parentMap.get(curr); 
		}
		path.addFirst(start);
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) 
	{
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		 if(!map.containsKey(start) || !map.containsKey(goal)) throw new IllegalArgumentException();

		//create map to store the each node's updated weight
		Map<GeographicPoint, Double> cityWeightUpdateMap = new HashMap<>();	
		
		LinkedList<MapNode> toVisit = new LinkedList<>();	
		
		Set<GeographicPoint> visited = new HashSet<>();		
	
		//set city weight of the start node to 0
		MapNode fromNode = new MapNode(start); 		
		fromNode.setCityWeight(0.0);		
		cityWeightUpdateMap.put(start, 0.0);		 
		toVisit.add(fromNode);
		
		Map<GeographicPoint, GeographicPoint> parentMap = dijkstraSearch(start, goal, nodeSearched, toVisit, visited, cityWeightUpdateMap);

		if(parentMap.size() == 0)
		{
			System.out.println("Route not found");
			return null;
		}
		//construct path
		List<GeographicPoint> route = constructPath(start, goal, parentMap);
		
		return route;  
	}
	
/**
 *  If it is in Rush hours of day :
 *  						Rush hour in the morning: From 8 a.m. to 10:30 a.m.
 *  						Rush hour in the evening: From 5 p.m to 7:30 p,m.
 *  during these time, if the comparison in the priority list is between a residential street and avenue
 *  then then the residential street will get 10% of its weight means 10% weight from its total weight
 *   will be decreased from the
 *  Residential street's weight
 * 
 * If its in the mid night:
 * 							Mid night : From 10 P.M. to 7 A.M.
 * During this time in comparison between avenue and residential street will get priority by 20% of its weight
 * 
 * 
 */
	//helper method dijkstraSearch
	private Map<GeographicPoint, GeographicPoint> dijkstraSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched, LinkedList<MapNode> toVisit, Set<GeographicPoint> visited,
																Map<GeographicPoint, Double> cityWeightUpdateMap)
	{
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
	
		while(!toVisit.isEmpty())
		{  
			//every time sort the list to get the minimum weight city at top
			Collections.sort(toVisit);
			
			//get top cityNode from the list
			MapNode  curr = toVisit.pollFirst(); 
			
			dijkstraNodeNumber++;
			
			// Hook for visualization
			nodeSearched.accept(curr.getLocation());
			
			//get the current city weight
			double currCityWeight = curr.getCityWeight();			
			
			//if the node has not been visited, add it to the visitedNode
			if(!visited.contains(curr.getLocation())) visited.add(curr.getLocation()); 
			
			//if searching city is found
			if(curr.getLocation().equals(goal)) break;
							
			//get the neighbor of the current city 
			ArrayList<MapEdge> neighbors = map.get(curr.getLocation()); 			 
			Iterator<MapEdge> it = neighbors.iterator();		
			
			while(it.hasNext())
			{ 
				MapEdge edge = it.next(); 
				
				//get city at the end of the edge
				MapNode neighborCity = edge.getNeighborCityName();
				
				if(!visited.contains(neighborCity.getLocation()) && !cityWeightUpdateMap.containsKey(neighborCity.getLocation()))
				{
					toVisit.add(neighborCity);
					//map the city and it's neighbor
					parentMap.put(neighborCity.getLocation(), curr.getLocation()); 
					
					//set the weight according to the day time
					double weight = setWeightAccordingToDayTime(currCityWeight, edge.getLength(), edge);

					neighborCity.setCityWeight(weight);
					cityWeightUpdateMap.put(neighborCity.getLocation(), weight);	
				}
			 
				if(!visited.contains(neighborCity.getLocation()) && cityWeightUpdateMap.containsKey(neighborCity.getLocation()))
				{
					double edgeWeight = edge.getLength();					
					//get neighbor (edge) City weight from the customized city weight map
					double neigbourCityWeight = cityWeightUpdateMap.get(neighborCity.getLocation());
				 
					if(neigbourCityWeight > currCityWeight + edgeWeight)
					{						
						//set the weight according to the day time
						double weight = setWeightAccordingToDayTime(currCityWeight, edge.getLength(), edge);
						
						neighborCity.setCityWeight(weight);						
						cityWeightUpdateMap.put(neighborCity.getLocation(), weight);							
						//update toVisit city list
						toVisit.add(neighborCity);
						//update map
						parentMap.put(neighborCity.getLocation(), curr.getLocation());
					}
				}
			} //end of inner while loop
		}//end of outer while loop 
		return parentMap;
	}
	
	//get node number visited by Dijkstra algorithm
	public int getDijkstraNodeNumber()
	{
		return dijkstraNodeNumber;
	}
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) 
	{
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if(!map.containsKey(start) || !map.containsKey(goal)) throw new IllegalArgumentException();

		//create map to store the each node's update weight
		Map<GeographicPoint, Double> cityWeightUpdateMap = new HashMap<>();		
		LinkedList<MapNode> toVisit = new LinkedList<>();		
		Set<GeographicPoint> visited = new HashSet<>();		
	
		//set city weight of the start node to 0
		MapNode fromNode = new MapNode(start); 		
		fromNode.setCityWeight(0.0);		
		cityWeightUpdateMap.put(start, 0.0);		 
		toVisit.add(fromNode);
		
		Map<GeographicPoint, GeographicPoint> parentMap = aStartSearch(start, goal, nodeSearched, toVisit, visited, cityWeightUpdateMap);

		if(parentMap.size() == 0)
		{
			System.out.println("Route not found");
			return null;
		}
		//construct path
		List<GeographicPoint> route = constructPath(start, goal, parentMap);
		
		return route;  
	 
	}
	//helper method aStarSearch
	private Map<GeographicPoint, GeographicPoint> aStartSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched, LinkedList<MapNode> toVisit, Set<GeographicPoint> visited,
																Map<GeographicPoint, Double> cityWeightUpdateMap)
	{
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		
		double distance = 0;
		
		while(!toVisit.isEmpty())
		{  
			//every time sort the list to get the minimum weight city at top
			Collections.sort(toVisit);
			
			//get top cityNode from the list
			MapNode curr = toVisit.pollFirst(); 
			
			aStarNodeNumber++;  
			
			// Hook for visualization
			nodeSearched.accept(curr.getLocation());
			//if the node has not been visited, add it to the visitedNode
			if(!visited.contains(curr.getLocation())) visited.add(curr.getLocation()); 
			
			//if searching city is found
			if(curr.getLocation().equals(goal)) break;						

			
			//get the neighbor of the current city 
			ArrayList<MapEdge> neighbors = map.get(curr.getLocation()); 	
			
			Iterator<MapEdge> it = neighbors.iterator();		
			
			while(it.hasNext())
			{ 
				MapEdge edge = it.next(); 
				
				//get the neighbor city at the end of the edge
				MapNode neighborCity = edge.getNeighborCityName();
				
				//get the distance from the neighbor city to the goal
				distance = neighborCity.getLocation().distance(goal);
				//get the current city weight
				double currCityWeight = 0.0;			
			
				//get city weight without the distance from the city weight map
				if(cityWeightUpdateMap.containsKey(curr.getLocation()))
				{
					currCityWeight = cityWeightUpdateMap.get(curr.getLocation());

				}
				else
						currCityWeight = curr.getCityWeight();
				
				//if the neighbor city has not been visited and city weight update map does not contain the city
				if(!visited.contains(neighborCity.getLocation()) && !cityWeightUpdateMap.containsKey(neighborCity.getLocation()))
				{
					//set the weight according to the day time
					double weight = setWeightAccordingToDayTime(currCityWeight, edge.getLength(), edge);
					//set city weight in list
					neighborCity.setCityWeight(weight + distance);

					toVisit.add(neighborCity); //add to the explore list
					
					parentMap.put(neighborCity.getLocation(), curr.getLocation()); //remember the visited nodes

					//map the city and it's neighbor
					cityWeightUpdateMap.put(neighborCity.getLocation(), currCityWeight + edge.getLength());	
					//System.out.println("cityWeightUpdateMap: " + cityWeightUpdateMap);
				}
			 
				if(!visited.contains(neighborCity.getLocation()) && cityWeightUpdateMap.containsKey(neighborCity.getLocation()))
				{
					double weight = currCityWeight + edge.getLength();		
					
					//get neighbor (edge) city weight from the customized city weight map
					double neigbourCityWeight = cityWeightUpdateMap.get(neighborCity.getLocation());
				 
					//if the unvisited neighbor has higher cost(weight) only than change the neighbor city weight
					if(neigbourCityWeight  > weight)
					{		
						weight = setWeightAccordingToDayTime(currCityWeight, edge.getLength(), edge);

						//update city weight with the newly founded weight and distance
						neighborCity.setCityWeight(weight + distance);	
						
						cityWeightUpdateMap.put(neighborCity.getLocation(), weight);
						
						//update toVisit city list
						toVisit.add(neighborCity);
						//update map
						parentMap.put(neighborCity.getLocation(), curr.getLocation());
					}
				}
			} //end of inner while loop
		}//end of outer while loop 
		
		return parentMap;
	}
	//get node number visited by aStar algorithm
	public int getAStarNodeNumber()
	{
		return aStarNodeNumber;
	}
	//set the weight according to the time
	private double setWeightAccordingToDayTime(double currCityWeight, double edgeLenght, MapEdge edge )
	{
		double weight = currCityWeight + edgeLenght;

		 /*  If it is Rush hours of day :
		 *  					Rush hour in the morning: From 8 a.m. to 10 a.m.
		  						Rush hour in the evening: From 5 p.m to 8 p,m.
		 */
		 
		if((edge.getRoadType().equals("Avenue") &&((currentTime >= 7 && currentTime <= 10) ||(currentTime >= 17 && currentTime <=20))))
				{
					weight = weight + weight/10;
				}

		 /* If its in the mid night:
		 * 							Mid night : From 22 P.M. to 7 A.M.
		 * During this time in comparison between avenue and residential street will get priority by 20% of its weight
		 */
		if((edge.getRoadType().equals("Avenue") &&((currentTime >= 22 && currentTime <= 7))))
		{
			weight = weight / weight/20;
		}						

		return weight;
	}

	public static void main(String[] args)
	{
		
		 MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		System.out.println("Dijkstra: " + simpleTestMap.getDijkstraNodeNumber() + " AStar: " + simpleTestMap.getAStarNodeNumber());

		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println("Dijkstra: " + testMap.getDijkstraNodeNumber() + " AStar: " + testMap.getAStarNodeNumber());

		MapGraph testMap2 = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap2);

		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap2.dijkstra(testStart,testEnd);
		testroute2 = testMap2.aStarSearch(testStart,testEnd);	
		
		System.out.println("Dijkstra: " + testMap2.getDijkstraNodeNumber() + " AStar: " + testMap2.getAStarNodeNumber());

		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
		System.out.println("Dijkstra: " + theMap.getDijkstraNodeNumber() + " AStar: " + theMap.getAStarNodeNumber());
	
	}
	
}
