package frc.robot.models;

// Define how the rocket data is to be stored, may eventually create get methods for access
public class PositionData {
    public int location;
    public String name;
    public String type; // Likely to be replaced by an enumeration
    public GamePieces gpType;

    public PositionData (int location, String name, String type){
        this.location = location;
        this.name = name;
        this.type = type;
    }

    public PositionData (int location, String name, GamePieces gpType){
        this.location = location;
        this.name = name;
        this.type = gpType.name();
        this.gpType = gpType;

    }

}