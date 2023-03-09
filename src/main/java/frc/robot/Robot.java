/* Copyright (c) FIRST and other WPILib contributors.
   Open Source Software; you can modify and/or share it under the terms of
   the WPILib BSD license file in the root directory of this project.
*/
/** 
 * @author Will Kam
 * @author Beavertronics
*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants.LogitechF130Controller;

//Drive motor control
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode; Not needed because we're using the WPI_Lib version
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; Arm is removed
//import edu.wpi.first.wpilibj.DigitalInput; used for arm limit switch but arm is removed

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

//import edu.wpi.first.wpilibj.ADXRS450_Gyro; RIP little gyro
//import edu.wpi.first.wpilibj.SPI; 

//import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.SerialPort.Port;

//import edu.wpi.first.cameraserver.CameraServer; //Not needed because jetson stuff is fancy

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController; RIP Xbox controller


public class Robot extends TimedRobot {

  //========================== ROBOT HARDWARE ==========================//

  //private final WPI_TalonFX m_liftarm_motor = new WPI_TalonFX(10);

  private final CANSparkMax usedForGrabbingEncoderR = new CANSparkMax(21, MotorType.kBrushless);
  private final MotorController m_rightmotors = 
  new MotorControllerGroup(
    usedForGrabbingEncoderR,
    new CANSparkMax(22, MotorType.kBrushless), 
    new CANSparkMax(23, MotorType.kBrushless));
    

  private final CANSparkMax usedForGrabbingEncoderL = new CANSparkMax(24, MotorType.kBrushless);
  private final MotorController m_leftmotors = 
  new MotorControllerGroup(
    usedForGrabbingEncoderL,
    new CANSparkMax(25, MotorType.kBrushless),
    new CANSparkMax(26, MotorType.kBrushless));

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftmotors, m_rightmotors);
  private final RelativeEncoder encoderL = usedForGrabbingEncoderL.getEncoder();
  private final RelativeEncoder encoderR = usedForGrabbingEncoderR.getEncoder();

  //private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  //private final AHRS navXIMU = new AHRS(Port.kUSB2); //USB2 is BOTTOM USB PORT!!!

  private final Solenoid shiftinator = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  private final Solenoid grabinator  = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  //================== DRIVER STATION HARDWARE (controllers) ==================//

  private final Joystick joyOperator= new Joystick(0);
  private final Joystick joyL = new Joystick(1);
  private final Joystick joyR = new Joystick(2);


  /*Auto Switching setup */
  enum Autos {
      LEAVE ("Just drive forwards to leave the Community"),
      SCORE_LEAVE ("Drive backwards to score, then drive forwards to leave the community."),
      NOTHING ("Just sit there");
  
      public String desc;
      private Autos(String desc) {
        this.desc= desc;
      }
    }
    private final SendableChooser<Autos> autoChooser = new SendableChooser<>();
    private Autos m_autoSelected;

  @Override
  public void robotInit() {

    autoChooser.setDefaultOption(Autos.NOTHING.desc, Autos.NOTHING);
    for (Autos auto : Autos.values()) {
      autoChooser.addOption(auto.desc, auto);
    }

    SmartDashboard.putData("Auto Choices:", autoChooser);

    shiftinator.set(true);

    m_leftmotors.setInverted(false); //Making sure they go the right way
    m_rightmotors.setInverted(true);

    //Don't zero the arm here, we're not allowed to move on startup

    //CameraServer.startAutomaticCapture(); //jetson stuff replaces this
  }

  /*Called every 20 ms, no matter the mode.*/
  @Override
  public void robotPeriodic() {
    //SmartDashboard.putNumber("NAVX Pitch", navXIMU.getPitch());
    //SmartDashboard.putNumber("NAVX Roll", navXIMU.getRoll());
    //SmartDashboard.putNumber("NAVX Yaw(Heading)", navXIMU.getYaw());
  }

  /**
   * Clamp value between +limit and -limit
   * @param value 
   * @param limit
   * @return Value, limited to + or - limit.
   */
  private double limitAbsWithSign(double value, double limit) {
    // If v is greater than l, use l instead of v
    // If v is less than -l, use -l instead of v
    // Otherwise v is between +-l so we're good.
    return Math.min(Math.abs(value), limit) * Math.signum(value);
  }

  private double getPosL() {
    if (shiftinator.get()) { //Low Gear
      return encoderL.getPosition() * Constants.Drive.INCHES_TO_LOW_GEAR_REVS;
    } else { //High Gear
      return encoderL.getPosition() * Constants.Drive.INCHES_TO_HIGH_GEAR_REVS;
    }
  }
  private double getPosR() {
    if (shiftinator.get()) { //Low Gear
      return encoderR.getPosition() * Constants.Drive.INCHES_TO_LOW_GEAR_REVS;
    } else { //High Gear
      return encoderR.getPosition() * Constants.Drive.INCHES_TO_HIGH_GEAR_REVS;
    }
  }
  private double getVelL() {
    if (shiftinator.get()) { //Low Gear
      return encoderL.getVelocity() * Constants.Drive.INCHES_TO_LOW_GEAR_REVS;
    } else { //High Gear
      return encoderL.getVelocity() * Constants.Drive.INCHES_TO_HIGH_GEAR_REVS;
    }
  }
  private double getVelR() {
    if (shiftinator.get()) { //Low Gear
      return encoderR.getVelocity() * Constants.Drive.INCHES_TO_LOW_GEAR_REVS;
    } else { //High Gear
      return encoderR.getVelocity() * Constants.Drive.INCHES_TO_HIGH_GEAR_REVS;
    }
  }

  double lastDriveErrL = 0;
  double lastDriveErrR = 0;

  private void tankDriveWithFF(double targetVelL, double targetVelR, double currentVelL, double currentVelR) {

    SmartDashboard.putNumber("Left Drive Target Vel", targetVelL);
    SmartDashboard.putNumber("Right Drive Target Vel", targetVelR);

    double targetAccelerationL = (targetVelL - currentVelL) / 1;
    //It will try to accelerate to hit targetVel in one second (dividing by Constants.DT would make it try to accelerate in one control frame)

    //double errSlopeL = (errL - lastDriveErrL) / Constants.DT;

    double targetAccelerationR = (targetVelR - currentVelR) / 1;
    //double errSlopeR = (errR - lastDriveErrR) / Constants.DT

    //Tank Drive, but using fancy feedforwards stuff
    double l = calcDriveFF(targetVelL, limitAbsWithSign(targetAccelerationL, Constants.Drive.MAX_ACC));
    double r = calcDriveFF(targetVelR, limitAbsWithSign(targetAccelerationR, Constants.Drive.MAX_ACC));

    m_rightmotors.setVoltage(limitAbsWithSign(r, Constants.Drive.DRIVE_V_LIMIT));
    m_leftmotors.setVoltage (limitAbsWithSign(l, Constants.Drive.DRIVE_V_LIMIT));
    m_drive.feed();

    SmartDashboard.putNumber("Left Drive Output Voltage", l);
    SmartDashboard.putNumber("Right Drive Output Voltage",r);
  }

  private boolean moveStraightWithEncoders(double distance, double targetSpeed) {

    double errL = getPosL() - distance;
    double errR = getPosR() - distance;

    if (Math.abs(errL) < Constants.Auto.POSITION_ACCURACY && Math.abs(errR) < Constants.Auto.POSITION_ACCURACY) {
      return true;
    }
    tankDriveWithFF(
      -targetSpeed, 
      -targetSpeed, 
      getVelL(), 
      getVelR()
    );
    return false;
  }

  private int move2FeetExactly(int step) {
    if (moveStraightWithEncoders(24, Constants.Auto.TRAVERSAL_SPEED)) {
      return 1;
    } else {
      return 0;
    }
  }


  double autoTimer = 0;
  private int scoreThenLeaveCommunity(int autoStepNumber) {
    switch(autoStepNumber) {
      case 0:
        if (System.currentTimeMillis() - autoTimer > Constants.Auto.SCORING_DRIVE_TIME * 1000) {
          autoTimer = System.currentTimeMillis();
          System.out.println("[auto] Scored Freight");
          return autoStepNumber + 1;
        } else {
          tankDriveWithFF(Constants.Auto.TRAVERSAL_SPEED,
            Constants.Auto.TRAVERSAL_SPEED,
            getVelL(),
            getVelR()  
          );
        }
        break;
      case 1:
        return leaveCommunity(autoStepNumber-1);
      case 2:
        return leaveCommunity(autoStepNumber-1);
    }
    return autoStepNumber;
  }

  /* 2 steps! */
  private int leaveCommunity(int autoStepNumber) {
    switch(autoStepNumber) {
      case 0:
        if (System.currentTimeMillis() - autoTimer > Constants.Auto.LEAVING_DRIVE_TIME * 1000 ) {
          System.out.println("[auto] Left Community!");

          return autoStepNumber + 1;
        } 
        tankDriveWithFF(
          -Constants.Auto.TRAVERSAL_SPEED, 
          -Constants.Auto.TRAVERSAL_SPEED,
          getVelL(), 
          getVelR()
        );
        
        break;
      case 1:
        tankDriveWithFF(-Constants.Auto.CHARGE_STATION_HOLD_VOLTAGE, -Constants.Auto.CHARGE_STATION_HOLD_VOLTAGE, 0, 0); //Slight voltage to not fall down 
        break;
    }
    return autoStepNumber;
  }

  int masterAutoProgressTracker = 0;

  @Override
  public void autonomousInit() {
    m_autoSelected = autoChooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected.desc);

    shiftinator.set(true); //Low gear for auto!
    grabinator.set(false); //Grabinator is inverted

    autoTimer = System.currentTimeMillis();
    masterAutoProgressTracker = 0;

    encoderL.setPosition(0);
    encoderR.setPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case NOTHING:     break;
      case LEAVE:       masterAutoProgressTracker = leaveCommunity(masterAutoProgressTracker); break;
      case SCORE_LEAVE: masterAutoProgressTracker = scoreThenLeaveCommunity(masterAutoProgressTracker); break;

      default:   throw new Error("Invalid Opmode!");
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("Teleop Initialized!");
  }


  private double calcDriveFF(double targetVel, double targetAcc) {
    return Constants.Drive.KS * Math.signum(targetVel) 
       + Constants.Drive.KV * targetVel 
       + Constants.Drive.KA * targetAcc;
  }

  @Override
  public void teleopPeriodic() {

    double joyl = joyL.getY();
    double joyr = joyR.getY();

    if (Math.abs(joyl) < 0.06) joyl = 0;
    if (Math.abs(joyr) < 0.06) joyr = 0;

    //joyl = (joyl * joyl) * Math.signum(joyl); //Input squaring
    //joyr = (joyr * joyr) * Math.signum(joyr); //Disabled; it was too much

    joyl *= Constants.Drive.TELE_SPEED_MULT;
    joyr *= Constants.Drive.TELE_SPEED_MULT;

    
    tankDriveWithFF(
      joyl, 
      joyr, 
      getVelL(),
      getVelR()
    );
    
    //Solonoids
    shiftinator.set(!joyR.getRawButton(2));
    if (joyOperator.getRawButton(LogitechF130Controller.kButtonX)) grabinator.set(true);
    if (joyOperator.getRawButton(LogitechF130Controller.kButtonY)) grabinator.set(false);
    
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_drive.stopMotor();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  private int testStep = 0;
  @Override
  public void testInit() {
    System.out.println("Test mode Init!");
    
  }

  @Override
  public void testPeriodic() {
    //teleopPeriodic();
    System.out.println(testStep);
    testStep = move2FeetExactly(testStep);
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
