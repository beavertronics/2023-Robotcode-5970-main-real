// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Authors: Will Kam
package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.LogitechF130Controller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode; Not needed because we're using the WPI_Lib version
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.cameraserver.CameraServer; Not needed because jetson stuff is fancy
//import edu.wpi.first.math.controller.PIDController; Just did it myself, they always overcomplicate things
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController; RIP Xbox controller
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

  /*Auto Switching setup */
  enum Autos {
    LEAVE ("Just drive forwards to leave the Community")
    ENGAGE_STATION ("Drive forwards onto the charge station and try to level it"),
    LEAVE_THEN_ENGAGE_STATION ("Drive over the charge station and leave the community, then reverse onto leveling the charge station.")
    NOTHING ("Just sit there");

    public String desc;
    private Autos(String desc) {
      this.desc= desc;
    }
  }
  private final SendableChooser<Autos> autoChooser = new SendableChooser<>();
  private Autos m_autoSelected;

  private final WPI_TalonFX m_liftarm_motor = new WPI_TalonFX(10);

  private final CANSparkMax usedForGrabbingEncoderL = new CANSparkMax(21, MotorType.kBrushless);
  private final CANSparkMax usedForGrabbingEncoderR = new CANSparkMax(24, MotorType.kBrushless);

  private final MotorController m_rightmotors = 
  new MotorControllerGroup(
    usedForGrabbingEncoderL,
    new CANSparkMax(22, MotorType.kBrushless), 
    new CANSparkMax(23, MotorType.kBrushless));

  private final MotorController m_leftmotors = 
  new MotorControllerGroup(
    usedForGrabbingEncoderR,
    new CANSparkMax(25, MotorType.kBrushless),
    new CANSparkMax(26, MotorType.kBrushless));

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftmotors, m_rightmotors);
  private final SparkMaxRelativeEncoder encoderL = usedForGrabbingEncoderL.getEncoder();
  private final SparkMaxRelativeEncoder encoderR = usedForGrabbingEncoderR.getEncoder();

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);


  private final Solenoid shiftinator = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  private final Solenoid grabinator  = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  private final Joystick joyOperator= new Joystick(0);
  private final Joystick joyL = new Joystick(1);
  private final Joystick joyR = new Joystick(2);

  private final DigitalInput armZeroSwitch = new DigitalInput(0);//THIS IS INVERTED- Is false when pressed

  //Cornvinience Functions
  private boolean getArmZeroSwitchHit() {
    return !armZeroSwitch.get();
  }
  private double getArmAngle() {
    SmartDashboard.putNumber("Arm Angle",m_liftarm_motor.getSelectedSensorPosition() * Constants.Arm.ENCODER_COUNTS_TO_ARM_DEGS );
    return m_liftarm_motor.getSelectedSensorPosition() * Constants.Arm.ENCODER_COUNTS_TO_ARM_DEGS;
  }

  private double autoStartTime = 0;

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
    //m_gyro.calibrate(); This happens on init and doing it here could mess things up

    //Don't zero the arm here, we're not allowed to move on startup

    //CameraServer.startAutomaticCapture(); jetson stuff replaces this
  }

  /*Called every 20 ms, no matter the mode.*/
  @Override
  public void robotPeriodic() {}

  private void limitAbsWithSign(v, l) {
    // If v is greater than l, use l instead of v
    // If v is less than -l, use -l instead of v
    // Otherwise v is between +-l so we're good.
    return Math.min(Math.abs(v), l) * Math.signum(v);
  }

  private void tankDriveWithFF(targetVelL, targetVelR, lastVelL, lastVelR) {
    //Tank Drive, but using fancy feedforwards stuff
    double l = calcDriveFF(targetVelL, limitAbsWithSign((targetVelL - lastVelL) / Constants.DT, Constants.Drive.MAX_ACC));
    double r = calcDriveFF(targetVelR, limitAbsWithSign((targetVelR - lastVelR) / Constants.DT, Constants.Drive.MAX_ACC));

    m_rightmotors.setVoltage(limitAbsWithSign(r, Constants.Drive.DRIVE_V_LIMIT));
    m_leftmotors.setVoltage (limitAbsWithSign(l, Constants.Drive.DRIVE_V_LIMIT));
    m_drive.feed(); //Required for some wierd reason.

    SmartDashboard.putNumber("Left Drive Voltage",l);
    SmartDashboard.putNumber("Right Drive Voltage",r);
  }

  private void approachChargeStation(boolean backwards) { //Drives forwards until it sees a change in gyro value to indicate it has gotten to the charge station
    if (Math.abs(m_gyro.getAngle()) > Constants.Auto.STATION_DETECTION_TILT) {
      return true;
    } 
    double targetSpeed = Constants.Auto.TRAVERSAL_SPEED;
    if (backwards) {
      targetSpeed *= -1;
    }
    tankDrivewithFF(targetSpeed,targetSpeed,encoderL.getVelocity(), encoderR.getVelocity());
    return false;
  }

  private void leaveChargeStation(boolean backwards) { //Drives forwards until it thinks it is off the charge station
    if (Math.abs(m_gyro.getAngle()) < Constants.Auto.STATION_EXIT_DETECTION_TILT) {
      return true;
    } 
    double targetSpeed = Constants.Auto.TRAVERSAL_SPEED;
    if (backwards) {
      targetSpeed *= -1;
    }
    tankDrivewithFF(targetSpeed,targetSpeed,encoderL.getVelocity(), encoderR.getVelocity());
    return false;
  }

  double lastTiltErr = 0;
  private void engageChargeStation() {
    //ONLY LOOP THIS ONCE YOU HAVE GOTTEN A true FROM approachChargeStation!
    double err = m_gyro.getAngle(); //We want gyro angle to be zero!
    double errSlope = (err - lastTiltErr) / Constants.DT;

    //Tell drivers once we think charge station is level
    SmartDashboard.putBoolean("Platform Good", Math.abs(err) < Constants.Auto.LEVELING_TOLERANCE);

    double p = err * Constants.Auto.LEVELING_KP + errSlope * Constants.Auto.LEVELING_KD;

    tankDriveWithFF(p, p, encoderL.getVelocity(), encoderR.getVelocity());

    
  }

  private int autoStepNumber = 0;
  private void justEngageChargeStation() {
    switch (autoStepNumber) {
      case 0: 
        boolean gotToChargeStation = approachChargeStation(false);
        if (gotToChargeStation) {
          SmartDashboard.putBoolean("Approached Charge Station", true);
          lastTiltErr = m_gyro.getAngle(); //Setup for engageChargeStation();
          autoStepNumber++;
        } 
        return;
      case 1:
        engageChargeStation();
        return;
    }
  }

  private void leaveCommunityThenEngageChargeStation() {
    switch (autoStepNumber) {
      case 0:
        if (approachChargeStation(false)) {
          autoStepNumber++;
        }
        return;
      case 1:
        if (leaveChargeStation(false)) {
          encoderL.setPosition(0);
          encoderR.setPosition(0);
          autoStepNumber++;
        } 
        return;
      case 2:
        if ((encoderL.getPosition() + encoderR.getPosition()) / 2 >= Constants.Auto.STATION_EXIT_EXTRA_DIST * Constants.Drive.ROTATIONS_PER_INCH) {
          leftCommunity = true;
          SmartDashboard.putBoolean("Left Community", true);
          autoStepNumber++;
        } else {
          tankDriveWithFF(Constants.Auto.TRAVERSAL_SPEED, Constants.Auto.TRAVERSAL_SPEED,encoderL.getVelocity(), encoderR.getVelocity() );
        }
        return;
      case 3:
        if (approachChargeStation(true)) {
          autoStepNumber++;
        }
        return;
      case 4:
        engageChargeStation();
    }
  }

  private void leaveCommunity() {
    switch(autoStepNumber) {
      case 0:
        if ((encoderL.getPosition() + encoderR.getPosition()) / 2 >= Constants.Auto.LEAVE_DIST *  Constants.Drive.ROTATIONS_PER_INCH) {
          leftCommunity = true;
          SmartDashboard.putBoolean("Left Community", true);
          autoStepNumber++;
        } else {
          tankDriveWithFF(Constants.Auto.TRAVERSAL_SPEED, Constants.Auto.TRAVERSAL_SPEED,encoderL.getVelocity(), encoderR.getVelocity() );
        }
        return;
      case 1:
        //We did it!!
        //Now would be a good time to flash some blinkenlights :)
        return;
    }
  }


  @Override
  public void autonomousInit() {
    m_autoSelected = autoChooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected.desc);
    switch (m_autoSelected) {
      case Autos.ENGAGE_STATION: 
        SmartDashboard.putBoolean("Approached Charge Station", false);
        break;
      case Autos.LEAVE_THEN_ENGAGE_STATION:
        SmartDashboard.putBoolean("Left Community", false);
        break;
      case Autos.LEAVE:
        SmartDashboard.putBoolean("Left Community", false);
        break;
    }
 
    autoStepNumber = 0; //If you don't start from square zero, things go very wrong (;

    autoStartTime = System.currentTimeMillis();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Auto Step",autoStepNumber);
    if (System.currentTimeMillis() - autoStartTime > 15 * 1000) {
      return; //Autonomous is done; wait for it to switch to teleop
    }
    switch (m_autoSelected) {
      case Autos.NOTHING: break;
      case Autos.LEAVE: leaveCommunity(); break;
      case Autos.ENGAGE_STATION: justEngageChargeStation();break; 
      case Autos.LEAVE_THEN_ENGAGE_STATION: leaveCommunityThenEngageChargeStation();break;
      default:   throw new Error("Invalid Opmode!");
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("Teleop Initialized!");
    weNeedToZero();
    m_liftarm_motor.setSelectedSensorPosition(0); //REMOVE AFTER TESTING
  }


  private double calcDriveFF(double targetVel, double targetAcc) {
    return Constants.Drive.KS * Math.signum(targetVel) 
      + Constants.Drive.KV * targetVel 
       + Constants.Drive.KA * targetAcc;
  }

  private double lastArmErr = 0; //Last 
  private double armI = 0; //Integral accumulator for arm PID
  private boolean needToZeroArm = false; //Whether or not we need to find the zero point w/ lim switch

  private void weNeedToZero() {
    needToZeroArm = true;
    SmartDashboard.putBoolean("Arm Good To Go", !needToZeroArm);
  }

  private void weNoLongerNeedToZero() {
    needToZeroArm = false;
    SmartDashboard.putBoolean("Arm Good To Go", !needToZeroArm);
  }


  private double calcArmPID(double setpoint, double pos) {

    double err = setpoint - pos; //P

    double errSlope = (err - lastArmErr) / Constants.DT; //D

    armI += err * Constants.DT; //I

    double voltage =  Constants.Arm.P * err +  Constants.Arm.I * armI + Constants.Arm.D * errSlope;
    lastArmErr = err;


    return voltage;
  }

  /** This function is called periodically during operator control. */

  double lastL = 0; //Last position of left control
  double lastR = 0;


  boolean inManualArmMode = false;
  double armTarget     = Constants.Arm.ANGLE_HOLD;
  double prevArmTarget = Constants.Arm.ANGLE_HOLD;

  @Override
  public void teleopPeriodic() {
    double joyl = joyL.getY();
    double joyr = joyR.getY();
    //joyl = (joyl * joyl) * Math.signum(joyl); //Input squaring
    //joyr = (joyr * joyr) * Math.signum(joyr); //Disabled; it was too much

    joyl *= Constants.Drive.TELE_SPEED_MULT;
    joyr *= Constants.Drive.TELE_SPEED_MULT;
    
    tankDriveWithFF(joyl, joyr, encoderL.getVelocity(), encoderR.getVelocity());


    double tilt = m_gyro.getAngle();
    SmartDashboard.putNumber("Gyro", tilt);

    //Solonoids
    shiftinator.set(!joyR.getRawButton(2));
    if (joyOperator.getRawButton(LogitechF130Controller.kButtonX)) grabinator.set(true);
    if (joyOperator.getRawButton(LogitechF130Controller.kButtonY)) grabinator.set(false);


    //Arm Lifting!!

    if (needToZeroArm) {
      //Arm Zeroing Ritual: Move backwards until we hit the limit switch, then stop and zero the encoder.
      if (getArmZeroSwitchHit()) {
        m_liftarm_motor.setSelectedSensorPosition(0); 
        m_liftarm_motor.set(0);
        
        weNoLongerNeedToZero();
      } else {
        m_liftarm_motor.setVoltage(Constants.Arm.ZEROING_VOLTAGE);
      }
    } else {

      //Once arm is zeroed, do things

      //Arm control works like this:
      //DPAD-UP will set the target level for the arm to be the HI position
      //DPAD-Right or DPAD Left will set the target position of the arm to the MID position
      //DPAD-Down will set the target pos to the HOLD position.

      //This is fine, but what if you want to manually adjust the position of the arm?
      //While holding the Left Bumper, The Left Stick (Up-And-Down) lets you send the arm up and down.
      //When you release the Left Bumper it will return to the last set target position.

      //Finally, X will cause the robot to try to GRAB a cube or cone, and
      // Y will try to DROP a cube or cone. Keep in mind the Pnumatics are not super fast!

      
      int dpadAngle = joyOperator.getPOV(0);
      switch (dpadAngle) {
        case 0:   armTarget = Constants.Arm.ANGLE_HI;   break;
        case 90:  armTarget = Constants.Arm.ANGLE_MID;  break;
        case 270: armTarget = Constants.Arm.ANGLE_MID;  break;
        case 180: armTarget = Constants.Arm.ANGLE_HOLD; break;
      }

      if (joyOperator.getRawButton(LogitechF130Controller.kButtonLB)) {
        if (!inManualArmMode) {
          inManualArmMode = true;
          prevArmTarget = armTarget;
        }
        armTarget += joyOperator.getRawAxis(LogitechF130Controller.kAxisLeftStickY);
      } else if (inManualArmMode) {
        armTarget = prevArmTarget;
        inManualArmMode = false;
      }

      
      //Automatic control
      double armV = calcArmPID(armTarget, getArmAngle());
      //double armV = joyOperator.getRawAxis(LogitechF130Controller.kAxisLeftStickY) * 5;
      SmartDashboard.putNumber("Arm Voltage", armV);
      m_liftarm_motor.setVoltage(armV);

    }
    
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_drive.stopMotor();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    System.out.println("Test mode Init!");
    teleopInit();
  }

  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
