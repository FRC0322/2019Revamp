package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.commands.*;
import frc.robot.utilities.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class defines the chassis subsystem and all of its parts.
 */
public class Chassis extends Subsystem {
    /*private WPI_TalonSRX leftFrontDriveMotor = Robot.chassisleftFrontDriveMotor;
    private WPI_TalonSRX leftRearDriveMotor = Robot.chassisleftRearDriveMotor;
    private SpeedControllerGroup leftSideDriveMotors = Robot.chassisleftSideDriveMotors;
    private WPI_TalonSRX rightFrontDriveMotor = Robot.chassisrightFrontDriveMotor;
    private WPI_TalonSRX rightRearDriveMotor = Robot.chassisrightRearDriveMotor;
    private SpeedControllerGroup rightSideDriveMotors = Robot.chassisrightSideDriveMotors;
    private DifferentialDrive robotDrive = Robot.chassisrobotDrive;*/

    private WPI_TalonSRX leftFrontDriveMotor;
    private WPI_TalonSRX leftRearDriveMotor ;
    private SpeedControllerGroup leftSideDriveMotors;
    private WPI_TalonSRX rightFrontDriveMotor;
    private WPI_TalonSRX rightRearDriveMotor ;
    private SpeedControllerGroup rightSideDriveMotors;
    private DifferentialDrive robotDrive;

     

   

    public Chassis(WPI_TalonSRX leftFrontDriveMotor , WPI_TalonSRX  leftRearDriveMotor  ,WPI_TalonSRX  rightFrontDriveMotor,WPI_TalonSRX rightRearDriveMotor,SpeedControllerGroup leftSideDriveMotors ,  SpeedControllerGroup rightSideDriveMotors, DifferentialDrive robotDrive  ){
      this.leftFrontDriveMotor = leftFrontDriveMotor;
      this.leftRearDriveMotor = leftRearDriveMotor;
      this.leftSideDriveMotors = leftSideDriveMotors;
      this.rightFrontDriveMotor = rightFrontDriveMotor;
      this.rightRearDriveMotor = rightRearDriveMotor;
      this.rightSideDriveMotors =rightSideDriveMotors;
      this.robotDrive = robotDrive;

    }


    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
      setDefaultCommand(new DriveWithJoystick());
    }
    
    @Override
    public void periodic() {
      // Put code here to be run every loop
    	dashboardUpdater();
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void driveWithJoystick(F310Controller driveStick) {
      //robotDrive.arcadeDrive(driveStick.getY(Hand.kLeft), -(driveStick.getX(Hand.kRight)), true);
      robotDrive.arcadeDrive(driveStick.getTriggerAxis(Hand.kLeft) - driveStick.getTriggerAxis(Hand.kRight), -(driveStick.getX(Hand.kLeft)), true);
    }
    
    public void autonDriveSystem(double xSpeed, double zRotation) {
    	robotDrive.arcadeDrive(-(xSpeed), zRotation);
    }

    public void pathfinderDriveSystem(double leftSpeed, double rightSpeed) {
    	robotDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void brakesOn() {
    	leftFrontDriveMotor.setNeutralMode(NeutralMode.Brake);
      leftRearDriveMotor.setNeutralMode(NeutralMode.Brake);
      rightFrontDriveMotor.setNeutralMode(NeutralMode.Brake);
      rightRearDriveMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public void brakesOff() {
    	leftFrontDriveMotor.setNeutralMode(NeutralMode.Coast);
      leftRearDriveMotor.setNeutralMode(NeutralMode.Coast);
      rightFrontDriveMotor.setNeutralMode(NeutralMode.Coast);
      rightRearDriveMotor.setNeutralMode(NeutralMode.Coast);
    }
/*    
    public void resetEncoders() {
    	leftFrontDriveMotor.getSensorCollection().setQuadraturePosition(0, 0);
    	leftRearDriveMotor.getSensorCollection().setQuadraturePosition(0, 0);
    	rightFrontDriveMotor.getSensorCollection().setQuadraturePosition(0, 0);
      rightRearDriveMotor.getSensorCollection().setQuadraturePosition(0, 0);
    }
    
    public void resetEncoders(int encoder) {
        switch(encoder)
    	{
    		case 0:
                leftFrontDriveMotor.getSensorCollection().setQuadraturePosition(0, 0);
    		case 1:
                leftRearDriveMotor.getSensorCollection().setQuadraturePosition(0, 0);
    		case 2:
                rightFrontDriveMotor.getSensorCollection().setQuadraturePosition(0, 0);
    		case 3:
                rightRearDriveMotor.getSensorCollection().setQuadraturePosition(0, 0);
    		default:
    			break;
    	}
    }

    public double getEncoderSpeed(int encoder) {
        switch(encoder)
    	{
    		case 0:
                return (double) leftFrontDriveMotor.getSensorCollection().getQuadratureVelocity();
    		case 1:
                return (double) leftRearDriveMotor.getSensorCollection().getQuadratureVelocity();
    		case 2:
                return (double) rightFrontDriveMotor.getSensorCollection().getQuadratureVelocity();
    		case 3:
                return (double) rightRearDriveMotor.getSensorCollection().getQuadratureVelocity();
    		default:
    			return 0.0;
    	}
    }

    public double getEncoderDistance(int encoder) {
        switch(encoder)
    	{
    		case 0:
                return (double) leftFrontDriveMotor.getSensorCollection().getQuadraturePosition();
    		case 1:
                return (double) leftRearDriveMotor.getSensorCollection().getQuadraturePosition();
    		case 2:
                return (double) rightFrontDriveMotor.getSensorCollection().getQuadraturePosition();
    		case 3:
                return (double) rightRearDriveMotor.getSensorCollection().getQuadraturePosition();
    		default:
    			return 0.0;
    	}
    }
*/
    public double getEncoderData(int encoder) {
    	switch(encoder)
    	{
    		case 0:
    			return (Robot.ENCODER_PULSE_DISTANCE / leftFrontDriveMotor.getSensorCollection().getQuadraturePosition());
    		
    		case 1:
    			return (Robot.ENCODER_PULSE_DISTANCE / leftRearDriveMotor.getSensorCollection().getQuadraturePosition());
    		
    		case 2:
    			return (Robot.ENCODER_PULSE_DISTANCE / rightFrontDriveMotor.getSensorCollection().getQuadraturePosition());
    		
    		case 3:
    			return (Robot.ENCODER_PULSE_DISTANCE / rightRearDriveMotor.getSensorCollection().getQuadraturePosition());
    		
    		default:
    			return 0.0;
    	}
    }

    public int getRawEncoderData(int encoder) {
    	switch(encoder)
    	{
    		case 0:
    			return leftFrontDriveMotor.getSensorCollection().getQuadraturePosition();
    		
    		case 1:
    			return leftRearDriveMotor.getSensorCollection().getQuadraturePosition();
    		
    		case 2:
    			return rightFrontDriveMotor.getSensorCollection().getQuadraturePosition();
    		
    		case 3:
    			return rightRearDriveMotor.getSensorCollection().getQuadraturePosition();
    		
    		default:
    			return 0;
    	}
    }
    
    public void dashboardUpdater() {
    	//SmartDashboard.putNumber("Left Front Distance: ", (double)this.getEncoderData(0));
    	//SmartDashboard.putNumber("Left Rear Distance: ", (double)this.getEncoderData(1));
    	//SmartDashboard.putNumber("Right Front Distance: ", (double)this.getEncoderData(2));
      //SmartDashboard.putNumber("Right Rear Distance: ", (double)this.getEncoderData(3));
      //SmartDashboard.putData("Left Side Drive Motors", leftSideDriveMotors);
      //SmartDashboard.putData("Right Side Drive Motors", rightSideDriveMotors);
      SmartDashboard.putData("Chassis", robotDrive);
    }

    public WPI_TalonSRX getLeftFrontDriveMotor() {
      return this.leftFrontDriveMotor;
}

public void setLeftFrontDriveMotor(WPI_TalonSRX leftFrontDriveMotor) {
      this.leftFrontDriveMotor = leftFrontDriveMotor;
}

public WPI_TalonSRX getLeftRearDriveMotor() {
      return this.leftRearDriveMotor;
}

public void setLeftRearDriveMotor(WPI_TalonSRX leftRearDriveMotor) {
      this.leftRearDriveMotor = leftRearDriveMotor;
}

public SpeedControllerGroup getLeftSideDriveMotors() {
      return this.leftSideDriveMotors;
}

public void setLeftSideDriveMotors(SpeedControllerGroup leftSideDriveMotors) {
      this.leftSideDriveMotors = leftSideDriveMotors;
}

public WPI_TalonSRX getRightFrontDriveMotor() {
      return this.rightFrontDriveMotor;
}

public void setRightFrontDriveMotor(WPI_TalonSRX rightFrontDriveMotor) {
      this.rightFrontDriveMotor = rightFrontDriveMotor;
}

public WPI_TalonSRX getRightRearDriveMotor() {
      return this.rightRearDriveMotor;
}

public void setRightRearDriveMotor(WPI_TalonSRX rightRearDriveMotor) {
      this.rightRearDriveMotor = rightRearDriveMotor;
}

public SpeedControllerGroup getRightSideDriveMotors() {
      return this.rightSideDriveMotors;
}

public void setRightSideDriveMotors(SpeedControllerGroup rightSideDriveMotors) {
      this.rightSideDriveMotors = rightSideDriveMotors;
}

public DifferentialDrive getRobotDrive() {
      return this.robotDrive;
}

public void setRobotDrive(DifferentialDrive robotDrive) {
      this.robotDrive = robotDrive;
}
}
