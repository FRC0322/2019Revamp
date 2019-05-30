/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.analog.adis16448.frc.*;
import frc.robot.utilities.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commands.*;
import frc.robot.subsystems.*;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static  Command autonomousCommand;

  
  public static  SendableChooser<String> chooser = new SendableChooser<>();
 
  public static final int CHASSIS_PWD_DIST_PANEL=0;
  /*chassisSensorspowerDistributionPanel = new PowerDistributionPanel(0);*/

  public static final int LED_CANIF_CONTRL=0;
  /*ledControlCANifier = new CANifier(0);*/
  
  public static final int CHASSIS_L_FRDM =1;
  public static final int CHASSIS_L_RRDM =2;
  public static final int CHASSIS_R_FRDM =3;
  public static final int CHASSIS_R_RRDM =4;
  /*chassisleftFrontDriveMotor = new WPI_TalonSRX(1);
  chassisleftRearDriveMotor = new WPI_TalonSRX(2);
  chassisrightFrontDriveMotor = new WPI_TalonSRX(3);
  chassisrightRearDriveMotor = new WPI_TalonSRX(4);*/

  public static final int ELEVATOR_FRDM =5;
  public static final int ELEVATOR_RRDM =6;
 /* elevatorFrontMotor = new WPI_TalonSRX(5);
  elevatorRearMotor = new WPI_TalonSRX(6);*/

  public static final int PIVOT_M =7;

  /*manipulatorPivotMotor = new WPI_TalonSRX(7);*/
  
  public static final int LEFT_WHEEL_M =1;
  public static final int RIGHT_WHEEL_M =2;
  
 /* manipulatorLeftWheelMotor = new WPI_VictorSPX(1);
  manipulatorRightWheelMotor = new WPI_VictorSPX(2);*/
  
  public static final int LIFT_L_FRDM =8;
  public static final int LIFT_L_RRDM =9;
  public static final int LIFT_R_FRDM =10;
  public static final int LIFT_R_RRDM =11;

  /*liftLeftFrontMotor = new WPI_TalonSRX(8);
  liftLeftRearMotor = new WPI_TalonSRX(9);
  liftRightFrontMotor = new WPI_TalonSRX(10);
 liftRightRearMotor = new WPI_TalonSRX(11);*/

  public static   WPI_TalonSRX chassisleftFrontDriveMotor;
  public static   WPI_TalonSRX chassisleftRearDriveMotor;
  public static   SpeedControllerGroup chassisleftSideDriveMotors;
  public static   WPI_TalonSRX chassisrightFrontDriveMotor;
  public static   WPI_TalonSRX chassisrightRearDriveMotor;
  public static   SpeedControllerGroup chassisrightSideDriveMotors;
  public static   DifferentialDrive chassisrobotDrive;
  public static   PowerDistributionPanel chassisSensorspowerDistributionPanel;
  public static   ADIS16448_IMU chassisSensorsIMU;
  public static   XGyro chassisSensorsXGyro;
  public static   YGyro chassisSensorsYGyro;
  public static   ZGyro chassisSensorsZGyro;
  public static   IMUAccelerometer chassisSensorsIMUAccelerometer;
  public static   SRXEncoder chassisSensorsLeftEncoder;
  public static   SRXEncoder chassisSensorsRightEncoder;
  public static   WPI_TalonSRX elevatorFrontMotor;
  public static   WPI_TalonSRX elevatorRearMotor;
  public static   SpeedControllerGroup elevatorMotors;
  public static   WPI_TalonSRX manipulatorPivotMotor;
  public static   WPI_VictorSPX manipulatorLeftWheelMotor;
  public static   WPI_VictorSPX manipulatorRightWheelMotor;
  public static   SpeedControllerGroup manipulatorWheelMotors;
  public static   CANifier ledControlCANifier;
  public static   WPI_TalonSRX liftLeftFrontMotor;
  public static   WPI_TalonSRX liftLeftRearMotor;
  public static   WPI_TalonSRX liftRightFrontMotor;
  public static   WPI_TalonSRX liftRightRearMotor;
  public static   SpeedControllerGroup liftMotors;
  public static   Preferences dashboardRobotPrefs;
  public static   NetworkTable limelightTable;
  public static   NetworkTableEntry limelighttx;
  public static   NetworkTableEntry limelightty;
  public static   NetworkTableEntry limelightta;
  public static   double limelightX, limelightY, limelightArea;
  public static   double autonDistance, autonRotation, autonSpeed, autonTime;
         //In Meters per Second
  public static   double redInt, greenInt, blueInt;
  public static   long ledBlinkRate;
  
 /*** ALL CONSTANTS ** */
  public static   final double WHEEL_DIAMETER = 8.375;
  public static   final double WHEEL_DIAMETER_METRIC = 0.212725;   //In Meters for FRC Pathfinder
  public static   final double ENCODER_GEAR_RATIO = 1.0;
  public static   final double GEAR_RATIO = 1.0;
  public static   final double FUDGE_FACTOR = 1.0;
  public static   final double ENCODER_PULSE_DISTANCE = Math.PI * WHEEL_DIAMETER / ENCODER_GEAR_RATIO / GEAR_RATIO * FUDGE_FACTOR;
  public static   final double AXLE_TRACK = 0.5398;              //In Meters for FRC Pathfinder
  public static   final double MAX_VELOCITY = 1.0; 
  
  public  static   OI oi;
  public static Chassis chassis;
  public  static ChassisSensors chassisSensors;
  public  static Dashboard dashboard;
  public  static  Debugger debugger;
  public  static  Elevator elevator;
  public  static  LEDControl ledControl;
  public  static  Lift lift;
  public  static  Limelight limelight;
  public  static  Manipulator manipulator;
  public  static DriverStation DS;
  public  static  UsbCamera rearCameraServer;

  

  

  
      
 public void mapSubsystemAddresses(){
    chassisSensorspowerDistributionPanel = new PowerDistributionPanel(CHASSIS_PWD_DIST_PANEL);
    ledControlCANifier = new CANifier(LED_CANIF_CONTRL);
    chassisleftFrontDriveMotor = new WPI_TalonSRX(CHASSIS_L_FRDM);
    chassisleftRearDriveMotor = new WPI_TalonSRX(CHASSIS_L_RRDM);
    chassisrightFrontDriveMotor = new WPI_TalonSRX(CHASSIS_R_FRDM);
    chassisrightRearDriveMotor = new WPI_TalonSRX(CHASSIS_R_RRDM);
    elevatorFrontMotor = new WPI_TalonSRX(ELEVATOR_FRDM);
    elevatorRearMotor = new WPI_TalonSRX(ELEVATOR_RRDM);
    manipulatorPivotMotor = new WPI_TalonSRX(PIVOT_M);  
    manipulatorLeftWheelMotor = new WPI_VictorSPX(LEFT_WHEEL_M);
    manipulatorRightWheelMotor = new WPI_VictorSPX(RIGHT_WHEEL_M);
    liftLeftFrontMotor = new WPI_TalonSRX(LIFT_L_FRDM);
    liftLeftRearMotor = new WPI_TalonSRX(LIFT_L_RRDM);
    liftRightFrontMotor = new WPI_TalonSRX(LIFT_R_FRDM);
    liftRightRearMotor = new WPI_TalonSRX(LIFT_R_RRDM);
 }


 public void buildChassis(){
 

  chassisleftSideDriveMotors = new SpeedControllerGroup(chassisleftFrontDriveMotor, chassisleftRearDriveMotor);
  chassisleftSideDriveMotors.setInverted(true);
  chassisleftSideDriveMotors.setName("Chassis", "leftSideDriveMotors");    
  
  chassisrightSideDriveMotors = new SpeedControllerGroup(chassisrightFrontDriveMotor, chassisrightRearDriveMotor);
  chassisrightSideDriveMotors.setInverted(true);
  chassisrightSideDriveMotors.setName("Chassis", "rightSideDriveMotors");
  chassisrobotDrive = new DifferentialDrive(chassisleftSideDriveMotors, chassisrightSideDriveMotors);
  
  
  chassisrobotDrive.setSafetyEnabled(true);
  chassisrobotDrive.setExpiration(5.0);
  chassisrobotDrive.setMaxOutput(1.0);
  chassisrobotDrive.setName("Chassis", "robotDrive");
  chassisSensorspowerDistributionPanel = new PowerDistributionPanel(0);
  chassisSensorspowerDistributionPanel.setName("ChassisSensors", "powerDistributionPanel");
  
 
  chassisSensorsIMU = new ADIS16448_IMU();
 
  chassisSensorsIMU.setName("ChassisSensors", "IMU");
  chassisSensorsXGyro = new XGyro();
  chassisSensorsXGyro.setName("ChassisSensors", "XGyro");
  chassisSensorsYGyro = new YGyro();
  chassisSensorsXGyro.setName("ChassisSensors", "YGyro");
  chassisSensorsZGyro = new ZGyro();
  chassisSensorsXGyro.setName("ChassisSensors", "ZGyro");
  chassisSensorsLeftEncoder = new SRXEncoder(chassisleftFrontDriveMotor);
  chassisSensorsLeftEncoder.setName("ChassisSensors", "LeftEncoder");
  chassisSensorsRightEncoder = new SRXEncoder(chassisrightFrontDriveMotor);
  chassisSensorsRightEncoder.setName("ChassisSensors", "RightEncoder");
  chassisSensorsIMUAccelerometer = new IMUAccelerometer();
  chassisSensorsIMUAccelerometer.setName("ChassisSensors", "IMUAccelerometer");
 
  chassis = new  Chassis(chassisleftFrontDriveMotor,chassisleftRearDriveMotor,chassisrightFrontDriveMotor, chassisrightRearDriveMotor, chassisleftSideDriveMotors,chassisrightSideDriveMotors,chassisrobotDrive);
  chassisSensors = new ChassisSensors();

 }
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

        mapSubsystemAddresses();
        buildChassis();

   

       
        /*elevatorFrontMotor.setNeutralMode(NeutralMode.Brake);
        elevatorRearMotor.setNeutralMode(NeutralMode.Brake);
        elevatorMotors = new SpeedControllerGroup(elevatorFrontMotor, elevatorRearMotor);
        elevatorMotors.setName("Elevator", "ElevatorMotors");
        manipulatorPivotMotor.setNeutralMode(NeutralMode.Brake);
        manipulatorPivotMotor.setName("Manipulator", "Pivot");
        
        manipulatorLeftWheelMotor.setNeutralMode(NeutralMode.Coast);
        manipulatorRightWheelMotor.setNeutralMode(NeutralMode.Coast);
        manipulatorWheelMotors = new SpeedControllerGroup(manipulatorLeftWheelMotor, manipulatorRightWheelMotor);
        manipulatorWheelMotors.setName("Manipulator", "WheelMotors");
*/

        redInt = 100.0;
        greenInt = 100.0;
        blueInt = 100.0;
        ledBlinkRate = 500;
        
        /*
        liftLeftFrontMotor.setNeutralMode(NeutralMode.Brake);
        liftLeftRearMotor.setNeutralMode(NeutralMode.Brake);
        liftRightFrontMotor.setNeutralMode(NeutralMode.Brake);
        liftRightRearMotor.setNeutralMode(NeutralMode.Brake);
        liftMotors = new SpeedControllerGroup(liftLeftFrontMotor, liftLeftRearMotor, liftRightFrontMotor, liftRightRearMotor);
        liftMotors.setName("Lift", "LiftMotors");
*/
        dashboardRobotPrefs = Preferences.getInstance();

        /*limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        limelighttx = limelightTable.getEntry("tx");
        limelightty = limelightTable.getEntry("ty");
        limelightta = limelightTable.getEntry("ta");*/

        limelightX = 0.0;
        limelightY = 0.0;
        limelightArea = 0.0;
        
        autonDistance = 36.0;
        autonSpeed = 0.5;
        autonRotation = 0.0;
        autonTime = 5.0;
        
        
        dashboard = new Dashboard();
        debugger = new Debugger();
       /* elevator = new Elevator();
        ledControl = new LEDControl();
        lift = new Lift();
        limelight = new Limelight();
        manipulator = new Manipulator();*/
        
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();
        DS = DriverStation.getInstance();
    
        //Setup Camera
       /* rearCameraServer = CameraServer.getInstance().startAutomaticCapture();
        rearCameraServer.setResolution(320, 180);
        rearCameraServer.setFPS(15);*/
    
        // Add commands to Autonomous Sendable Chooser
        chooser.setDefaultOption("Do Nothing", "Do Nothing");
        SmartDashboard.putData("Auto mode", chooser);
    
       /* Scheduler.getInstance().add(new InitializeDashboard());
        Scheduler.getInstance().add(new InitializeSensors());
        Scheduler.getInstance().add(new LimelightCamera());*/
        Scheduler.getInstance().run();




  }



  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
     // dont add back Scheduler.getInstance().add(new GetRoboPrefs());
   // Scheduler.getInstance().add(new ReadLimelight());
    Scheduler.getInstance().add(new DashboardOutput());
    Scheduler.getInstance().run();
  }
  
  @Override
  public void disabledInit(){
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard.
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    switch (chooser.getSelected()) {
      case "Do Nothing":  autonomousCommand = new DoNothing();
      break;
      
      default:            autonomousCommand = new DoNothing();
      break;
    }
    
    if (autonomousCommand != null) autonomousCommand.start();
  }

  @Override
  public void testInit() {
  }

  
  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }



  public WPI_TalonSRX getChassisleftFrontDriveMotor() {
    return this.chassisleftFrontDriveMotor;
  }

  public void setChassisleftFrontDriveMotor(WPI_TalonSRX chassisleftFrontDriveMotor) {
    this.chassisleftFrontDriveMotor = chassisleftFrontDriveMotor;
  }

  public WPI_TalonSRX getChassisleftRearDriveMotor() {
    return this.chassisleftRearDriveMotor;
  }

  public void setChassisleftRearDriveMotor(WPI_TalonSRX chassisleftRearDriveMotor) {
    this.chassisleftRearDriveMotor = chassisleftRearDriveMotor;
  }

  public SpeedControllerGroup getChassisleftSideDriveMotors() {
    return this.chassisleftSideDriveMotors;
  }

  public void setChassisleftSideDriveMotors(SpeedControllerGroup chassisleftSideDriveMotors) {
    this.chassisleftSideDriveMotors = chassisleftSideDriveMotors;
  }

  public WPI_TalonSRX getChassisrightFrontDriveMotor() {
    return this.chassisrightFrontDriveMotor;
  }

  public void setChassisrightFrontDriveMotor(WPI_TalonSRX chassisrightFrontDriveMotor) {
    this.chassisrightFrontDriveMotor = chassisrightFrontDriveMotor;
  }

  public WPI_TalonSRX getChassisrightRearDriveMotor() {
    return this.chassisrightRearDriveMotor;
  }

  public void setChassisrightRearDriveMotor(WPI_TalonSRX chassisrightRearDriveMotor) {
    this.chassisrightRearDriveMotor = chassisrightRearDriveMotor;
  }

  public SpeedControllerGroup getChassisrightSideDriveMotors() {
    return this.chassisrightSideDriveMotors;
  }

  public void setChassisrightSideDriveMotors(SpeedControllerGroup chassisrightSideDriveMotors) {
    this.chassisrightSideDriveMotors = chassisrightSideDriveMotors;
  }

  public DifferentialDrive getChassisrobotDrive() {
    return this.chassisrobotDrive;
  }

  public void setChassisrobotDrive(DifferentialDrive chassisrobotDrive) {
    this.chassisrobotDrive = chassisrobotDrive;
  }

  public PowerDistributionPanel getChassisSensorspowerDistributionPanel() {
    return this.chassisSensorspowerDistributionPanel;
  }

  public void setChassisSensorspowerDistributionPanel(PowerDistributionPanel chassisSensorspowerDistributionPanel) {
    this.chassisSensorspowerDistributionPanel = chassisSensorspowerDistributionPanel;
  }

  public ADIS16448_IMU getChassisSensorsIMU() {
    return this.chassisSensorsIMU;
  }

  public void setChassisSensorsIMU(ADIS16448_IMU chassisSensorsIMU) {
    this.chassisSensorsIMU = chassisSensorsIMU;
  }

  public XGyro getChassisSensorsXGyro() {
    return this.chassisSensorsXGyro;
  }

  public void setChassisSensorsXGyro(XGyro chassisSensorsXGyro) {
    this.chassisSensorsXGyro = chassisSensorsXGyro;
  }

  public YGyro getChassisSensorsYGyro() {
    return this.chassisSensorsYGyro;
  }

  public void setChassisSensorsYGyro(YGyro chassisSensorsYGyro) {
    this.chassisSensorsYGyro = chassisSensorsYGyro;
  }

  public ZGyro getChassisSensorsZGyro() {
    return this.chassisSensorsZGyro;
  }

  public void setChassisSensorsZGyro(ZGyro chassisSensorsZGyro) {
    this.chassisSensorsZGyro = chassisSensorsZGyro;
  }

  public IMUAccelerometer getChassisSensorsIMUAccelerometer() {
    return this.chassisSensorsIMUAccelerometer;
  }

  public void setChassisSensorsIMUAccelerometer(IMUAccelerometer chassisSensorsIMUAccelerometer) {
    this.chassisSensorsIMUAccelerometer = chassisSensorsIMUAccelerometer;
  }

  public SRXEncoder getChassisSensorsLeftEncoder() {
    return this.chassisSensorsLeftEncoder;
  }

  public void setChassisSensorsLeftEncoder(SRXEncoder chassisSensorsLeftEncoder) {
    this.chassisSensorsLeftEncoder = chassisSensorsLeftEncoder;
  }

  public SRXEncoder getChassisSensorsRightEncoder() {
    return this.chassisSensorsRightEncoder;
  }

  public void setChassisSensorsRightEncoder(SRXEncoder chassisSensorsRightEncoder) {
    this.chassisSensorsRightEncoder = chassisSensorsRightEncoder;
  }

  public WPI_TalonSRX getElevatorFrontMotor() {
    return this.elevatorFrontMotor;
  }

  public void setElevatorFrontMotor(WPI_TalonSRX elevatorFrontMotor) {
    this.elevatorFrontMotor = elevatorFrontMotor;
  }

  public WPI_TalonSRX getElevatorRearMotor() {
    return this.elevatorRearMotor;
  }

  public void setElevatorRearMotor(WPI_TalonSRX elevatorRearMotor) {
    this.elevatorRearMotor = elevatorRearMotor;
  }

  public SpeedControllerGroup getElevatorMotors() {
    return this.elevatorMotors;
  }

  public void setElevatorMotors(SpeedControllerGroup elevatorMotors) {
    this.elevatorMotors = elevatorMotors;
  }

  public WPI_TalonSRX getManipulatorPivotMotor() {
    return this.manipulatorPivotMotor;
  }

  public void setManipulatorPivotMotor(WPI_TalonSRX manipulatorPivotMotor) {
    this.manipulatorPivotMotor = manipulatorPivotMotor;
  }

  public WPI_VictorSPX getManipulatorLeftWheelMotor() {
    return this.manipulatorLeftWheelMotor;
  }

  public void setManipulatorLeftWheelMotor(WPI_VictorSPX manipulatorLeftWheelMotor) {
    this.manipulatorLeftWheelMotor = manipulatorLeftWheelMotor;
  }

  public WPI_VictorSPX getManipulatorRightWheelMotor() {
    return this.manipulatorRightWheelMotor;
  }

  public void setManipulatorRightWheelMotor(WPI_VictorSPX manipulatorRightWheelMotor) {
    this.manipulatorRightWheelMotor = manipulatorRightWheelMotor;
  }

  public SpeedControllerGroup getManipulatorWheelMotors() {
    return this.manipulatorWheelMotors;
  }

  public void setManipulatorWheelMotors(SpeedControllerGroup manipulatorWheelMotors) {
    this.manipulatorWheelMotors = manipulatorWheelMotors;
  }

  public CANifier getLedControlCANifier() {
    return this.ledControlCANifier;
  }

  public void setLedControlCANifier(CANifier ledControlCANifier) {
    this.ledControlCANifier = ledControlCANifier;
  }

  public WPI_TalonSRX getLiftLeftFrontMotor() {
    return this.liftLeftFrontMotor;
  }

  public void setLiftLeftFrontMotor(WPI_TalonSRX liftLeftFrontMotor) {
    this.liftLeftFrontMotor = liftLeftFrontMotor;
  }

  public WPI_TalonSRX getLiftLeftRearMotor() {
    return this.liftLeftRearMotor;
  }

  public void setLiftLeftRearMotor(WPI_TalonSRX liftLeftRearMotor) {
    this.liftLeftRearMotor = liftLeftRearMotor;
  }

  public WPI_TalonSRX getLiftRightFrontMotor() {
    return this.liftRightFrontMotor;
  }

  public void setLiftRightFrontMotor(WPI_TalonSRX liftRightFrontMotor) {
    this.liftRightFrontMotor = liftRightFrontMotor;
  }

  public WPI_TalonSRX getLiftRightRearMotor() {
    return this.liftRightRearMotor;
  }

  public void setLiftRightRearMotor(WPI_TalonSRX liftRightRearMotor) {
    this.liftRightRearMotor = liftRightRearMotor;
  }

  public SpeedControllerGroup getLiftMotors() {
    return this.liftMotors;
  }

  public void setLiftMotors(SpeedControllerGroup liftMotors) {
    this.liftMotors = liftMotors;
  }

  public Preferences getDashboardRobotPrefs() {
    return this.dashboardRobotPrefs;
  }

  public void setDashboardRobotPrefs(Preferences dashboardRobotPrefs) {
    this.dashboardRobotPrefs = dashboardRobotPrefs;
  }

  public NetworkTable getLimelightTable() {
    return this.limelightTable;
  }

  public void setLimelightTable(NetworkTable limelightTable) {
    this.limelightTable = limelightTable;
  }

  public NetworkTableEntry getLimelighttx() {
    return this.limelighttx;
  }

  public void setLimelighttx(NetworkTableEntry limelighttx) {
    this.limelighttx = limelighttx;
  }

  public NetworkTableEntry getLimelightty() {
    return this.limelightty;
  }

  public void setLimelightty(NetworkTableEntry limelightty) {
    this.limelightty = limelightty;
  }

  public NetworkTableEntry getLimelightta() {
    return this.limelightta;
  }

  public void setLimelightta(NetworkTableEntry limelightta) {
    this.limelightta = limelightta;
  }

  public double getLimelightX() {
    return this.limelightX;
  }

  public void setLimelightX(double limelightX) {
    this.limelightX = limelightX;
  }

  public double getLimelightY() {
    return this.limelightY;
  }

  public void setLimelightY(double limelightY) {
    this.limelightY = limelightY;
  }

  public double getLimelightArea() {
    return this.limelightArea;
  }

  public void setLimelightArea(double limelightArea) {
    this.limelightArea = limelightArea;
  }

  public double getAutonDistance() {
    return this.autonDistance;
  }

  public void setAutonDistance(double autonDistance) {
    this.autonDistance = autonDistance;
  }

  public double getAutonRotation() {
    return this.autonRotation;
  }

  public void setAutonRotation(double autonRotation) {
    this.autonRotation = autonRotation;
  }

  public double getAutonSpeed() {
    return this.autonSpeed;
  }

  public void setAutonSpeed(double autonSpeed) {
    this.autonSpeed = autonSpeed;
  }

  public double getAutonTime() {
    return this.autonTime;
  }

  public void setAutonTime(double autonTime) {
    this.autonTime = autonTime;
  }

  public double getWHEEL_DIAMETER() {
    return this.WHEEL_DIAMETER;
  }


  public double getWHEEL_DIAMETER_METRIC() {
    return this.WHEEL_DIAMETER_METRIC;
  }


  public double getENCODER_GEAR_RATIO() {
    return this.ENCODER_GEAR_RATIO;
  }


  public double getGEAR_RATIO() {
    return this.GEAR_RATIO;
  }


  public double getFUDGE_FACTOR() {
    return this.FUDGE_FACTOR;
  }


  public double getENCODER_PULSE_DISTANCE() {
    return this.ENCODER_PULSE_DISTANCE;
  }


  public double getAXLE_TRACK() {
    return this.AXLE_TRACK;
  }


  public double getMAX_VELOCITY() {
    return this.MAX_VELOCITY;
  }


  public double getRedInt() {
    return this.redInt;
  }

  public void setRedInt(double redInt) {
    this.redInt = redInt;
  }

  public double getGreenInt() {
    return this.greenInt;
  }

  public void setGreenInt(double greenInt) {
    this.greenInt = greenInt;
  }

  public double getBlueInt() {
    return this.blueInt;
  }

  public void setBlueInt(double blueInt) {
    this.blueInt = blueInt;
  }

  public long getLedBlinkRate() {
    return this.ledBlinkRate;
  }

  public void setLedBlinkRate(long ledBlinkRate) {
    this.ledBlinkRate = ledBlinkRate;
  }
  
  public OI getOi() {
    return this.oi;
  }

  public void setOi(OI oi) {
    this.oi = oi;
  }

  public Chassis getChassis() {
    return this.chassis;
  }

  public void setChassis(Chassis chassis) {
    this.chassis = chassis;
  }

  public ChassisSensors getChassisSensors() {
    return this.chassisSensors;
  }

  public void setChassisSensors(ChassisSensors chassisSensors) {
    this.chassisSensors = chassisSensors;
  }

  public Dashboard getDashboard() {
    return this.dashboard;
  }

  public void setDashboard(Dashboard dashboard) {
    this.dashboard = dashboard;
  }

  public Debugger getDebugger() {
    return this.debugger;
  }

  public void setDebugger(Debugger debugger) {
    this.debugger = debugger;
  }

  public Elevator getElevator() {
    return this.elevator;
  }

  public void setElevator(Elevator elevator) {
    this.elevator = elevator;
  }

  public LEDControl getLedControl() {
    return this.ledControl;
  }

  public void setLedControl(LEDControl ledControl) {
    this.ledControl = ledControl;
  }

  public Lift getLift() {
    return this.lift;
  }

  public void setLift(Lift lift) {
    this.lift = lift;
  }

  public Limelight getLimelight() {
    return this.limelight;
  }

  public void setLimelight(Limelight limelight) {
    this.limelight = limelight;
  }

  public Manipulator getManipulator() {
    return this.manipulator;
  }

  public void setManipulator(Manipulator manipulator) {
    this.manipulator = manipulator;
  }

  public DriverStation getDS() {
    return this.DS;
  }

  public void setDS(DriverStation DS) {
    this.DS = DS;
  }

  public UsbCamera getRearCameraServer() {
    return this.rearCameraServer;
  }

  public void setRearCameraServer(UsbCamera rearCameraServer) {
    this.rearCameraServer = rearCameraServer;
  }

  public Command getAutonomousCommand() {
    return this.autonomousCommand;
  }

  public void setAutonomousCommand(Command autonomousCommand) {
    this.autonomousCommand = autonomousCommand;
  }

  public SendableChooser<String> getChooser() {
    return this.chooser;
  }

  public void setChooser(SendableChooser<String> chooser) {
    this.chooser = chooser;
  }
}
