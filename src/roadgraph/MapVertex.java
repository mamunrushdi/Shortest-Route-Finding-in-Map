package roadgraph;


import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class MapVertex 
{
	private GeographicPoint location; //each vertex is created for each GeographiPoint object
	
	private List<MapEdge> edges; //each vertex may have more than one edge
	
	//creating a MapVertex with a GeographicPoint
	public MapVertex(GeographicPoint location)
	{
		this.location = location;
		edges = new ArrayList<>(); //initialize each vertex with an empty edge list
	}
	
	//set neighbors 
	/*
	 * this method create an edge with the destination Vertex and road's information
	 */
	public void setEdges(GeographicPoint to, String roadName,
			String roadType, double length)
	{
		MapEdge edge = new MapEdge(to, roadName,roadType, length);
		edges.add(edge);
	}
	
	//return the list of edges of this vertex
	public List<MapEdge> getEdges()
	{
		return edges;
	}
	
	//get the location of vertex
	public GeographicPoint getLocation()
	{
		return location;
	}
	
	//Override the hashCode code regarding the MapVertex objects
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((location == null) ? 0 : location.hashCode());
		return result;
	}
	//Override the equals method regarding the MapVertex object
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		MapVertex other = (MapVertex) obj;
		if (location == null) {
			if (other.location != null)
				return false;
		} else if (!location.equals(other.location))
			return false;
		return true;
	}
	
	//Override the toString method to print the Map vertex
	@Override
	public String toString()
	{
		String ret = "";
		
		GeographicPoint location = this.getLocation();
		 ret = location.getX() + "," + location.getY();
		return ret;
	}
}
