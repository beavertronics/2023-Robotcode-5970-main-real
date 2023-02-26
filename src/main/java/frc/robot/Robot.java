// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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


	//Clickysticks are next!

  /*Auto Switching setup */
  enum Autos {
    MSP ("Just move forwards and shoot and pray"),
    Nothing ("Just sit there");

    public String desc;
    private Autos(String desc) {
      this.desc= desc;
    }
  }
  private final SendableChooser<Autos> autoChooser = new SendableChooser<>();
  private Autos m_autoSelected;

  private final TalonFX m_liftarm_motor = new TalonFX(10);

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

  //private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);


  private final Solenoid shiftinator = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  private final Solenoid grabinator  = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  private final Joystick joyOperator= new Joystick(0);
  private final Joystick joyL = new Joystick(1);
  private final Joystick joyR = new Joystick(2);

  private final DigitalInput armZeroSwitch = new DigitalInput(0);//THIS IS INVERTED- Is false when pressed

  private boolean getArmZeroSwitchHit() {
    return !armZeroSwitch.get();
  }

  private double autoStartTime = 0;

  //private PIDController armPidController = new PIDController(1,0 ,0);

  @Override
  public void robotInit() {

    autoChooser.setDefaultOption(Autos.Nothing.desc, Autos.Nothing);
    for (Autos auto : Autos.values()) {
      autoChooser.addOption(auto.desc, auto);
    }
    SmartDashboard.putData("Auto Choices:", autoChooser);

    m_leftmotors.setInverted(false); //Making sure they go the right way
    m_rightmotors.setInverted(true);
    //m_gyro.calibrate();

    //CameraServer.startAutomaticCapture();
  }

  /*Called every 20 ms, no matter the mode.*/
  @Override
  public void robotPeriodic() {}


  @Override
  public void autonomousInit() {
    m_autoSelected = autoChooser.getSelected();
 
    System.out.println("Auto selected: " + m_autoSelected.desc);
    autoStartTime = System.currentTimeMillis();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case Nothing: break;
      case MSP: throw new Error("This opmode is unimplemented!");
      default:   throw new Error("Invalid Opmode!");
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("Teleop Initialized!");
    needToZeroArm = true;
  }


  private double calcDriveFF(double targetVel, double targetAcc) {
    return Constants.Drive.KS * Math.signum(targetVel) + Constants.Drive.KV * targetVel + Constants.Drive.KA * targetAcc;
  }

  private double lastArmErr = 0; //Last 
  private double armI = 0; //Integral accumulator for arm PID

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

  boolean needToZeroArm = false;

  @Override
  public void teleopPeriodic() {

    double joyl = joyL.getY();
    double joyr = joyR.getY();
    
    double l = calcDriveFF(joyl, Math.min(Math.abs(joyl-lastL), Constants.Drive.MAX_ACC) * Math.signum(joyl-lastL));
    double r = calcDriveFF(joyr, Math.min(Math.abs(joyr-lastR), Constants.Drive.MAX_ACC) * Math.signum(joyr-lastR));

    lastL = joyl;
    lastR = joyr;


    //Tank Drive1-
    m_rightmotors.setVoltage(Math.min(Math.abs(r), Constants.Drive.DRIVE_V_LIMIT) * Math.signum(r));
    m_leftmotors.setVoltage(Math.min(Math.abs(l), Constants.Drive.DRIVE_V_LIMIT) * Math.signum(l));
    m_drive.feed();
    SmartDashboard.putNumber("Left Drive Voltage",l);
    SmartDashboard.putNumber("Right Drive Voltage",r);


    //double turningValue = m_gyro.getAngle();
    //SmartDashboard.putNumber("Gyro", turningValue);

    //Solonoids
    shiftinator.set(joyR.getTrigger());
    grabinator.set(joyOperator.getRawButton(Constants.Controllers.kGamepadButtonLT));

    //Arm Lifting!!

    //Arm Zeroing Ritual

    /*if (needToZeroArm) {
      if (getArmZeroSwitchHit()) {
        m_liftarm_motor.setSelectedSensorPosition(0);
        m_liftarm_motor.set(TalonFXControlMode.PercentOutput, 0);
        needToZeroArm = false;
      } else {
        m_liftarm_motor.set(TalonFXControlMode.Current, Constants.Arm.ZEROING_VOLTAGE);
      }
    } else {*/

      double armV = joyOperator.getRawAxis(Constants.Controllers.kGamepadAxisLeftStickX);//calcArmPID(m_liftarm_motor.getSelectedSensorPosition() / 2048 / 400, joyOperator.getRawAxis(Constants.Controllers.kGamepadAxisLeftStickX));
      //if (joyOperator.getRawButton(Constants.Controllers.kGamepadButtonX)){
        SmartDashboard.putNumber("Arm Power", armV);
        m_liftarm_motor.set(TalonFXControlMode.PercentOutput, armV);
      //} else {
      //  m_liftarm_motor.set(TalonFXControlMode.PercentOutput,0);
     // }

    //}
    
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
