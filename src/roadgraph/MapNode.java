package roadgraph;
/**
 * @author MD Al Mamunur Rashid (Mamun)
 * This class represent a city (which is a location object) and it's weight
 */
import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode>
{
	private GeographicPoint location;
	private double cityWeight;// = Double.POSITIVE_INFINITY;
	
	
	public MapNode(GeographicPoint location)
	{
		this.location = location;
		cityWeight = Double.POSITIVE_INFINITY; 
	} 
	 
	public double getCityWeight()
	{
		return cityWeight;
	}
	
	public void setCityWeight(Double weight)
	{
		cityWeight = weight;
	}
	
	public GeographicPoint getLocation()
	{
		return location;
	}
	
	public String toString()
	{
		String ret = "";		
		ret = "(" + location.getX() + "," +location.getY() +")" + ":" + cityWeight;
		return ret;
	}
	/* 
	 * rewriting the hascode() method
	 */
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((location == null) ? 0 : location.hashCode());
		long temp;
		temp = Double.doubleToLongBits(cityWeight);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		return result;
	}
	/* 
	 * rewriting the equals() method
	  */
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		MapNode other = (MapNode) obj;
		if (location == null) {
			if (other.location != null)
				return false;
		} else if (!location.equals(other.location))
			return false;
		if (Double.doubleToLongBits(cityWeight) != Double.doubleToLongBits(other.cityWeight))
			return false;
		return true;
	}
	@Override
	public int compareTo(MapNode o) 
	{ 
		return Double.compare(this.cityWeight, o.cityWeight);
//		if(this.cityName.compareTo(o.cityName) < 1) return -1;
//		if(this.cityName.compareTo(o.cityName) > 1) return 1;
//		return 0;if(this.cityName.compareTo(o.cityName) < 1) return -1;
//		if(this.cityName.compareTo(o.cityName) > 1) return 1;
//		return 0;
	}
}
