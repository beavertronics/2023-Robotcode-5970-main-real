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

//=================Large TODOs:
//TODO: Refactor to use a command based architechture (Will make auto programming 10 times easier, but will take a long time)
//TODO: Properly sort out drive directions
//TODO: Create proper Logitech F130 class so I don't have to deal with it manually
//TODO: Make a better system to allow all Constants to be tuned automatically from the dashboard
  /*We should be able to tune parameters in smartdashboard until they are good, then copy them into the code
    and make it so we can't accidentally mess them up.
    I also want to make sure we have this info as a display to help debug hardware problems:
      -Drivetrain target speeds (L & R)
      -Drivetrain measured speeds (L & R)
      -Drivetrain output voltages (L & R)
      -Drivetrain measured current draw (L & R)
      -Gyro data (pitch at least)
      -States of all solonoids in a row
      -Compressor state (is it running or not?)
      -Arm position info
      -Readout for what stage the autonomous is in
      -Measured Position in autonomous (L & R? )
      -Beavertronics logo in some empty space
  */
//TODO: Use Simulation and unit testing, so silly code mistakes get caught before breaking the robot.
//TODO: Motion profiling and more advanced auto control stuff, so our robot can be *smooth*
//TODO: (1.) Detect when hardware is missing, (2.) tell you about it, and (3.) work to the best of its ablility without the missing part (Way, way harder than you think)
//TODO: Educate the younglings on all that I do




public class Robot extends TimedRobot {

  //========================== ROBOT HARDWARE ==========================//

  //private final WPI_TalonFX m_liftarm_motor = new WPI_TalonFX(10);


  private final CANSparkMax usedForGrabbingEncoderR = new CANSparkMax(21, MotorType.kBrushless);

    //TODO: Possibly refactor this with most of the spark maxes .follow()ing a leader
    // Then again, might make things more annoying
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
  private final Solenoid liftinator  = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

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

    m_leftmotors.setInverted(false); //TODO: Check that I'm actually going the right way
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
      return encoderL.getPosition() * Constants.Drive.LOW_GEAR_REVS_TO_INCHES;
    } else { //High Gear
      return encoderL.getPosition() * Constants.Drive.HIGH_GEAR_REVS_TO_INCHES;
    }
  }
  private double getPosR() {
    if (shiftinator.get()) { //Low Gear
      return encoderR.getPosition() * Constants.Drive.LOW_GEAR_REVS_TO_INCHES;
    } else { //High Gear
      return encoderR.getPosition() * Constants.Drive.HIGH_GEAR_REVS_TO_INCHES;
    }
  }
  private double getVelL() {
    if (shiftinator.get()) { //Low Gear
      return encoderL.getVelocity() * Constants.Drive.LOW_GEAR_REVS_TO_INCHES;
    } else { //High Gear
      return encoderL.getVelocity() * Constants.Drive.HIGH_GEAR_REVS_TO_INCHES;
    }
  }
  private double getVelR() {
    if (shiftinator.get()) { //Low Gear
      return encoderR.getVelocity() * Constants.Drive.LOW_GEAR_REVS_TO_INCHES;
    } else { //High Gear
      return encoderR.getVelocity() * Constants.Drive.HIGH_GEAR_REVS_TO_INCHES;
    }
  }

  /*
  private double getCorrectedPitch() {
    return navXIMU.getPitch() + Constants.Auto.GYRO_ANGLE_ADJUST
  }

  //TODO: Stop repeating the same PID code everwhere!
  double lastLevelingErr = 0;
  double levelingI = 0;

  //Uses a PID loop to get the charge station level.
  //Returns 0 unless it is level, in which case it returns 1.
  
  private int engageChargeStation() {
    double err = getCorrectedPitch(); //TODO: May need to be inverted

    if (Math.abs(err) < Constants.Auto.LEVELING_ACCURACY) {
      return 1;
    }

    double errSlope = (err - lastLevelingErr) / Constants.DT;

    levelingI += err * Constants.DT;

    double drivePower = err * Constants.Auto.LEVELING_P + levelingI * Constants.Auto.LEVELING_I + errSlope * Constants.Auto.LEVELING_D;

    tankDriveWithFF(drivePower,drivePower, getVelL(), getVelR()); //TODO: May need to be inverted
    return 0;
  }

  */

  private void tankDriveWithFF(double targetVelL, double targetVelR, double currentVelL, double currentVelR) {

    SmartDashboard.putNumber("Left Drive Target Vel", targetVelL);
    SmartDashboard.putNumber("Right Drive Target Vel", targetVelR);

    double targetAccelerationL = targetVelL - currentVelL; //TODO: This being a wierd PID loop in diguise is confusing
    double targetAccelerationR = targetVelR - currentVelR;

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

    double targetSpeedL = targetSpeed;
    double targetSpeedR = targetSpeed;

    if (Math.abs(errL) < Constants.Auto.POSITION_ACCURACY) {
      targetSpeedL = 0;
    }
    if (Math.abs(errR) < Constants.Auto.POSITION_ACCURACY) {
      targetSpeedR = 0;
    }

    if (targetSpeedL == 0 && targetSpeedR == 0) {
      return true;
    }
    tankDriveWithFF( //Warning, sharp accelerations!
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
/* 
  private int levelPlatform(int autoStepNumber) {
    switch(autoStepNumber) {
      case 0:
        if (System.currentTimeMillis() - autoTimer > Constants.Auto.LEVELING1_APPROACH_TIME) {
          autoTimer = System.currentTimeMillis();
          System.out.println("[auto] Approached Platform!");
          return 1;
        } else {
          tankDriveWithFF(Constants.Auto.STAION_APPROACH_SPEED,
          Constants.Auto.STAION_APPROACH_SPEED,
          getVelL(),
          getVelR()  
        );
        }
      case 1: 
        return 1 + engageChargeStation();
      case 2:
        tankDriveWithFF(0,
          0,
          getVelL(), //<- If things are acting up, replace getVelL and getVelR with 0 (just here not anywhere else).
          getVelR() 
        );
        return 2;
    }
  }
*/
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
        return leaveCommunity(autoStepNumber-1) + 1;
      case 2:
        return leaveCommunity(autoStepNumber-1) + 1;
    }
    return autoStepNumber;
  }

  /* 2 steps! */
  private int leaveCommunity(int autoStepNumber) {
    switch(autoStepNumber) {
      case 0:
        if (System.currentTimeMillis() - autoTimer > Constants.Auto.LEAVING_DRIVE_TIME * 1000 ) {
          System.out.println("[auto] Left Community!");

          return 1;
        } 
        tankDriveWithFF(
          -Constants.Auto.TRAVERSAL_SPEED, 
          -Constants.Auto.TRAVERSAL_SPEED,
          getVelL(), 
          getVelR()
        );
        return 0;
      case 1:
        
        //tankDriveWithFF(-Constants.Auto.CHARGE_STATION_HOLD_VOLTAGE, -Constants.Auto.CHARGE_STATION_HOLD_VOLTAGE, 0, 0); //Slight voltage to not fall down 
        return 1;
    }
    return autoStepNumber;
  }

  int masterAutoProgressTracker = 0;

  @Override
  public void autonomousInit() {
    m_autoSelected = autoChooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected.desc);

    shiftinator.set(false); //Low gear for auto!
    grabinator.set(true); //Grabinator is inverted
    liftinator.set(false);

    autoTimer = System.currentTimeMillis();
    masterAutoProgressTracker = 0;

    encoderL.setPosition(0);
    encoderR.setPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) { //TODO: Redo the auto step control system to use commands instead (Warning: Learning curve)
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


  //TODO: Consider whether or not to stop using custom feedforwards (keeping it makes code easier to explain, but could have bugs)
  private double calcDriveFF(double targetVel, double targetAcc) {
    return Constants.Drive.KS * Math.signum(targetVel) 
       + Constants.Drive.KV * targetVel 
       + Constants.Drive.KA * targetAcc;
  }

  @Override
  public void teleopPeriodic() {

    //============TELEOP DRIVING!!
    double joyl = joyL.getY();
    double joyr = joyR.getY();


    if (Math.abs(joyl) < 0.06) joyl = 0;
    if (Math.abs(joyr) < 0.06) joyr = 0;

    /*
    joyl = (joyl * joyl) * Math.signum(joyl); //Input squaring
    joyr = (joyr * joyr) * Math.signum(joyr);
    Disabled; it was too much */

    if (joyL.getTrigger()) {
      joyl *= Constants.Drive.TELE_FAST_SPEED_MULT;
      joyr *= Constants.Drive.TELE_FAST_SPEED_MULT;
    } else {

      joyl *= Constants.Drive.TELE_NORM_SPEED_MULT;
      joyr *= Constants.Drive.TELE_NORM_SPEED_MULT;
    }

    
    tankDriveWithFF(
      joyl, 
      joyr, 
      getVelL(),
      getVelR()
    );
    
    //Solonoids

    //==============SHIFTING!!
    shiftinator.set(!joyR.getRawButton(2));

    //==============GRABBY CLAW!!!!
    if (joyOperator.getRawButton(LogitechF130Controller.kButtonX)) grabinator.set(true);
    if (joyOperator.getRawButton(LogitechF130Controller.kButtonY)) grabinator.set(false);
    
    if (joyOperator.getRawButtonPressed(LogitechF130Controller.kButtonA)) liftinator.toggle();


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
