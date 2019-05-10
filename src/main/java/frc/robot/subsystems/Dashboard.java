/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Robot;
import frc.robot.commands.*;

/**
 * This subsystem describes a portion of the controls for the dashboard.
 */
public class Dashboard extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void getRoboPrefs() {
    Robot.autonDistance = Robot.dashboardRobotPrefs.getDouble("Autonomous Distance", Robot.autonDistance);
  	Robot.autonSpeed = Robot.dashboardRobotPrefs.getDouble("Autonomous Speed", Robot.autonSpeed);
	  Robot.autonRotation = Robot.dashboardRobotPrefs.getDouble("Autonomous Rotation", Robot.autonRotation);
    Robot.autonTime = Robot.dashboardRobotPrefs.getDouble("Autonomous Time", Robot.autonTime);
    SmartDashboard.putNumber("Autonomous Speed", Robot.autonSpeed);
    SmartDashboard.putNumber("Autonomous Rotation", Robot.autonRotation);
    SmartDashboard.putNumber("Autonomous Distance", Robot.autonDistance);
    SmartDashboard.putNumber("Autonomous Time", Robot.autonTime);
  }

  public void initializeDashboard() {
    SmartDashboard.putData("DriveWithJoystick", new DriveWithJoystick());
    SmartDashboard.putData("Brakes", new Brakes());
    SmartDashboard.putData("ResetEncoders", new ResetEncoders());
    SmartDashboard.putData("ResetGyro", new ResetGyro());
    SmartDashboard.putData("DoNothing", new DoNothing());
  }

  public void dashboardOutput() {
    SmartDashboard.putNumber("LimelightX", Robot.limelightX);
    SmartDashboard.putNumber("LimelightY", Robot.limelightY);
    SmartDashboard.putNumber("LimelightArea", Robot.limelightArea);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DashboardOutput());
  }
}
