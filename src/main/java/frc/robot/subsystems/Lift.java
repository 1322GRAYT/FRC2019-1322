package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commands.TC_LiftMotor;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Class: LIFT - Controls the Telescopic Lift System that the Claw that
 * manipulates the PowerCubes mounts to, and the hook used to hang the
 * robot mounts to as well.
 */
public class Lift extends Subsystem {

	enum LiftActn
	  {
	  Raise, Lower;
	  }
	
	// Create New Instances of all required IO Objects	
    private TalonSRX lift1 = new TalonSRX(RobotMap.LIFT_1); 				//Lift Motor 1
    private TalonSRX lift2 = new TalonSRX(RobotMap.LIFT_2); 				//Lift Motor 2
    private Solenoid shiftO = new Solenoid(RobotMap.LIFT_SHIFT_O); 			//Shift Solenoid Valve 1
    private Solenoid shiftC = new Solenoid(RobotMap.LIFT_SHIFT_C); 			//Shift Solenoid Valve 2
    private Solenoid liftJam = new Solenoid(RobotMap.LIFT_JAM); 			//Lift Jammer 
    private DigitalInput lowSen = new DigitalInput(RobotMap.LOW_LIFT); 		//Sensor at bottom of lift
    private DigitalInput midSen = new DigitalInput(RobotMap.MID_LIFT); 		//Sensor at middle of lift
    private DigitalInput highSen = new DigitalInput(RobotMap.HIGH_LIFT); 	//Sensor at top of lift


    /** Method: shiftLiftLow - Engage Lift Motor Low Speed Gear */
    public void shiftLiftLow(){
    	shiftO.set(true);
    	shiftC.set(false);
    }

    /** Method: shiftLiftHigh - Engage Lift Motor High Speed Gear */
    public void shiftLiftHigh(){
    	shiftO.set(false);
    	shiftC.set(true);
    }
    
    /**
     * Method: getLowGear - Verify Lift Motor is in Low Speed Gear  
     * @return: Is Low Gear Engaged
     */
    private boolean getLowGear() {
    	return !shiftO.get() && shiftC.get();
    }
    
    /**
     * Method: getHighGear - Verify Lift Motor is in Low Speed Gear  
     * @return: Is High Gear engaged
     */
    private boolean getHighGear() {
    	return shiftO.get() && !shiftC.get();
    }
    
    /**
     * Method: getLowSen - Return the state of the Lift Low Position Sensor - N/C Switch
     * @return: Low Lift Sensor Value (default true)
     */
    public boolean getLowSen() {
    	return lowSen.get();
    }
    
    /**
     * Method: getMidSen - Return the state of the Lift Mid Position Sensor - N/C Switch
     * @return: Mid Lift Sensor Value (default true)
     */
    public boolean getMidSen() {
    	return midSen.get();
    }

    /**
     * Method: getHighSen - Return the state of the Lift High Position Sensor - N/C Switch
     * @return: High Lift Sensor Value (default true)
     */
    public boolean getHighSen() {
    	return highSen.get();
    }
        
    /** Method: disengageJammer - Release the Lift Gear Block mechanism that 
     *  locks the lift in position to allow the lift to move.  */    
    public void disengageJammer() {
    	liftJam.set(true);
    }
    
    /** Method: engageJammer - Engage the Lift Gear Block mechanism that 
     *  locks the lift in position to prevent the lift from back-driving.  */    
    public void engageJammer() {
    	liftJam.set(true);
    }
    
    /**
     * Method: setSpeed - Set the Motor Speed of the Lift System Motor    
     * @param: speed What speed you want the lift to go
     */
    public void setSpeed(double speed) {
    	double upPower = dzify(speed);
    	if((lowSen.get() && highSen.get()) || 
    	   (!lowSen.get() && upPower > 0.31) || 
    	   (!highSen.get() && upPower < -0.31)) 
    	  {
    	  if(speed != 0.0)disengageJammer();
    	  lift1.set(ControlMode.PercentOutput, upPower);
    	  lift2.set(ControlMode.PercentOutput, upPower);
    	  if(getLowGear() && speed == 0.0) engageJammer(); //Check If We are in low gear
    	  } 	
    }
   
    /**
     * Method: dzify - Applies a DeadZone around a threshold limit.
	 * @param: A double between -1 and 1
	 * @return: A double that is now been deadzoned
	 */
    private double dzify(double value) {
		double deadzone = 0.1;
		if(value > deadzone || value < -deadzone)
		  {
		  return value;
		  }
		return 0.0;
	}
 
    /**
     * Method: initDefaultCommand - Sets the Default Command for the Subsystem.      
	 * to TC_LiftMotor so that it is ALWAYS running throughout Tele-Op.
	 */
    public void initDefaultCommand() {
       //setDefaultCommand(new TC_LiftMotor());
    }
}
