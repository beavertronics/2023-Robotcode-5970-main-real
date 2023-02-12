// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /*Auto Switching setup */
  private static final String kDefaultAuto = "Just move forwards and shoot and pray";
  //private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  //private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /*
  Motor Drivers: 
  

  */


  private final MotorController m_rightmotors = 
  new MotorControllerGroup(
    new CANSparkMax(21, MotorType.kBrushless), 
    new CANSparkMax(22, MotorType.kBrushless), 
    new CANSparkMax(23, MotorType.kBrushless));

  private final MotorController m_leftmotors = 
  new MotorControllerGroup(
    new CANSparkMax(24, MotorType.kBrushless),
    new CANSparkMax(25, MotorType.kBrushless),
    new CANSparkMax(26, MotorType.kBrushless));

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftmotors, m_rightmotors);

  private final TalonFX m_liftarm_motor = new TalonFX(10);

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);


  private SimpleMotorFeedforward ffR;
  private SimpleMotorFeedforward ffL;


  private final Solenoid shiftinator = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

  private final Joystick joyL = new Joystick(0);
  private final Joystick joyR = new Joystick(1);

  private final XboxController joyXbox = new XboxController(3);
  private double autoStartTime = 0;

  private double kS, kV, kA, kMaxVel, kMaxAcc;

  private final String[] things = {kDefaultAuto};


  public void getParamsFromSmartDashboard() {
    boolean changed = false;
    if (SmartDashboard.getNumber("kS", 0) != kS) {changed = true; kS = SmartDashboard.getNumber("kS", 0);};
    if (SmartDashboard.getNumber("kV", 0) != kV) {changed = true; kV = SmartDashboard.getNumber("kV", 0);};
    if (SmartDashboard.getNumber("kA", 0) != kA) {changed = true; kA = SmartDashboard.getNumber("kA", 0);};
    if (changed) {
      ffL = new SimpleMotorFeedforward(kS, kV, kA);
      ffR = new SimpleMotorFeedforward(kS, kV, kA);
    }

  }

  @Override
  public void robotInit() {
    //m_chooser.setDefaultOption(kDefaultAuto + " (Default Auto)", kDefaultAuto);
    //m_chooser.addOption("Do Nothing???", "");
    SmartDashboard.putStringArray("Auto List", things);

    kS = 0;
    kV = 0;
    kA = 0;

    ffL = new SimpleMotorFeedforward(kS, kV, kA);
    ffR = new SimpleMotorFeedforward(kS, kV, kA);



    m_leftmotors.setInverted(true); //Making sure they go the right way
    m_rightmotors.setInverted(false);
    m_gyro.calibrate();

    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autoStartTime = System.currentTimeMillis();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      /*case kCustomAuto:
        // Put custom auto code here
        break;*/
      case kDefaultAuto:
      default:
      System.out.println("This auto was removed");
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("Teleop Initialized!");
  }

  public double[] motorStats = {0,0,0,0}; 
  public double kDt = 0.02;


  public double profiledTargetVelL = 0;
  public double profiledTargetVelR = 0;
  public double dt = 0.02;

  //public double profiledTargetAcc = 0;
  //public double kJerk = 1;

  private double calcVelRamp(double targetVel, double smoothedVel) {

    double desiredAccel = (targetVel - smoothedVel)/dt;
    if (Math.abs(desiredAccel) > kMaxAcc) {
      desiredAccel = Math.copySign(kMaxAcc, desiredAccel);
    }
    smoothedVel += desiredAccel * dt; 
    return smoothedVel;
  }

  /** This function is called periodically during operator control. */

  @Override
  public void teleopPeriodic() {

    profiledTargetVelL = calcVelRamp(joyL.getY(), profiledTargetVelL);
    profiledTargetVelR = calcVelRamp(joyR.getY(), profiledTargetVelR);
    
    double l = joyL.getY();//ffL.calculate(profiledTargetVelL);
    double r = joyR.getY();//ffR.calculate(profiledTargetVelR);

    motorStats[0] = l;
    motorStats[1] = r;
    motorStats[2] = l;
    motorStats[3] = r;


    //Tank Drive1-
    m_drive.tankDrive(l, r);
    SmartDashboard.putNumber("RobotDriveL",l);
    SmartDashboard.putNumber("RobotDriveR",r);


    double turningValue = m_gyro.getAngle();
    //System.out.println(turningValue);

    SmartDashboard.putNumber("Gyro", turningValue);



    //Shifting
    if (joyR.getTriggerPressed()) {
      shiftinator.toggle();
    }

    //Arm Lifting!!
    if (joyXbox.getBButton()){
      m_liftarm_motor.set(joyXbox.getLeftY());
    } else {
      m_liftarm_motor.set(0);
    }
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    //m_drive.stopMotor();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    System.out.println("Test Mode Was Removed!");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
    //disabled
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
