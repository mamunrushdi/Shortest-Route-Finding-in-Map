package roadgraph;
/**
 * @author MD Al Mamunur Rashid (Mamun)
 * 
 * This class represent the connecting road of location, road name and road type
 */
import geography.GeographicPoint;

public class MapEdge 
{
	private MapNode neighborCityNode;
	private double edgeWeight; //
	private String roadName;
	private String roadType; 
	
	/**
	 * 
	 * In our map road could be three type
	 *  1. Bypass
	 *  2. Avenue
	 *  3. Residential
	 */
	
	//if a road edge is type of Bypass, it will be automatically be awarded 50 less weight
	//which mean in the length of this road will be decreased by 50 unit, thus bypass will get priority
	
	
	public MapEdge(GeographicPoint to, String roadName,
			String roadType, double length)
	{
		this.neighborCityNode = new MapNode(to);
		this.edgeWeight = length;
		this.roadName = roadName;
		this.roadType = roadType;
		
		if(this.roadType.equals("Bypass"))
		{
			this.edgeWeight = this.edgeWeight - this.edgeWeight/10;;

		}
	}
	
	public MapNode getNeighborCityName()
	{
		return neighborCityNode;
	}
	public String getRoadName()
	{
		return roadName;
	}
	
	public String getRoadType()
	{
		return roadType;
	}
	//get distance
	public double getLength()
	{
		return edgeWeight;
	}
	
	//get location
	public GeographicPoint getLocation()
	{
		return neighborCityNode.getLocation();
	}
	public String toString()
	{
		String ret = "";
		
		ret = neighborCityNode + " " +"edgeWeight: "+ edgeWeight + " " + roadName + " " + roadName;
		
		return ret;
	}
	
	
}
