package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.Robot;
import frc.robot.commands.*;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class defines the LED control system.
 */
public class LEDControl extends Subsystem {
	
	private final CANifier canifier = Robot.ledControlCANifier;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new SetLED());
    }

    public void setRGB(double redIntensity, double greenIntensity, double blueIntensity, long blinkRate) throws InterruptedException {
    	canifier.setLEDOutput(redIntensity, LEDChannel.LEDChannelA);
    	canifier.setLEDOutput(greenIntensity, LEDChannel.LEDChannelB);
    	canifier.setLEDOutput(blueIntensity, LEDChannel.LEDChannelC);
    	Thread.sleep(blinkRate);
    	canifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
    	canifier.setLEDOutput(0.0, LEDChannel.LEDChannelB);
    	canifier.setLEDOutput(0.0, LEDChannel.LEDChannelC);
    }
    
    public void automaticLEDSetter() throws InterruptedException {
    	if(Robot.DS.isDisabled()) 				Robot.ledBlinkRate = 500;
    	else if(Robot.DS.isAutonomous())		Robot.ledBlinkRate = 200;
    	else if(Robot.DS.isOperatorControl())	Robot.ledBlinkRate = 100;
    	else									Robot.ledBlinkRate = 0;
    	
    	if(Robot.DS.getAlliance() == DriverStation.Alliance.Red) {
    		Robot.redInt = 100.0;
    		Robot.greenInt = 0.0;
    		Robot.blueInt = 0.0;
    	}
    	else if(Robot.DS.getAlliance() == DriverStation.Alliance.Blue) {
    		Robot.redInt = 0.0;
    		Robot.greenInt = 0.0;
    		Robot.blueInt = 100.0;
    	}
    	else if(Robot.DS.getAlliance() == DriverStation.Alliance.Invalid) {
    		Robot.redInt = 100.0;
    		Robot.greenInt = 0.0;
    		Robot.blueInt = 100.0;
    	}
    	else {
    		Robot.redInt = 0.0;
    		Robot.greenInt = 100.0;
    		Robot.blueInt = 0.0;
    	}
    	
    	setRGB(Robot.redInt, Robot.greenInt, Robot.blueInt, Robot.ledBlinkRate);
    }
}

