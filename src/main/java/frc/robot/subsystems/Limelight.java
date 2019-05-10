/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.*;

/**
 * This subsystem describes a small portion of the controls for the Limelight Vision
 * system.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void readLimelight() {
    Robot.limelightX = Robot.limelighttx.getDouble(0.0);
    Robot.limelightY = Robot.limelightty.getDouble(0.0);
    Robot.limelightArea = Robot.limelightta.getDouble(0.0);
  }

  public void setCamMode(int mode) {
    switch (mode) {
      case 0:   Robot.limelightTable.getEntry("camMode").setNumber(0);
      break;
      
      case 1:   Robot.limelightTable.getEntry("camMode").setNumber(1);
      break;

      default:  Robot.limelightTable.getEntry("camMode").setNumber(0);
      break;
    }
  }

  public void setPipeline(int pipe) {
    switch (pipe) {
      case 0: Robot.limelightTable.getEntry("pipeline").setNumber(0);
      break;

      case 1: Robot.limelightTable.getEntry("pipeline").setNumber(1);
      break;

      case 2: Robot.limelightTable.getEntry("pipeline").setNumber(2);
      break;

      case 3: Robot.limelightTable.getEntry("pipeline").setNumber(3);
      break;

      case 4: Robot.limelightTable.getEntry("pipeline").setNumber(4);
      break;

      case 5: Robot.limelightTable.getEntry("pipeline").setNumber(5);
      break;

      case 6: Robot.limelightTable.getEntry("pipeline").setNumber(6);
      break;

      case 7: Robot.limelightTable.getEntry("pipeline").setNumber(7);
      break;

      case 8: Robot.limelightTable.getEntry("pipeline").setNumber(8);
      break;

      case 9: Robot.limelightTable.getEntry("pipeline").setNumber(9);
      break;
    }
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ReadLimelight());
  }
}
