package frc.robot.models;

// Define how the rocket data is to be stored, may eventually create get methods for access
public class PositionData {
    public int location;
    public String name;
    public String type; // Likely to be replaced by an enumeration

    public PositionData (int location, String name, String type){
        this.location = location;
        this.name = name;
        this.type = type;
    }
}