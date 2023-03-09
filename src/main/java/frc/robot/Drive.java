package frc.robot;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class Drive extends SubsystemBase {
    public Drive() {};

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

    private double calcDriveFF(double targetVel, double targetAcc) {
        return Constants.Drive.KS * Math.signum(targetVel) 
           + Constants.Drive.KV * targetVel 
           + Constants.Drive.KA * targetAcc;
      }

    public void tankDriveWithEncoders(double targetVelL, double targetVelR) {

        m_leftmotors.setName("Drivetrain", "Left");55

        SmartDashboard.putNumber("Left Drive Target Vel", targetVelL);
        SmartDashboard.putNumber("Right Drive Target Vel", targetVelR);
    
        double errL = targetVelL - lastVelL;
        double errSlopeL = (errL - lastDriveErrL) / Constants.DT;
    
        double errR = targetVelR - lastVelR;
        double errSlopeR = (errR - lastDriveErrR) / Constants.DT;
    
        SmartDashboard.putNumber("Left Drive Vel Err", errL);
        SmartDashboard.putNumber("Right Drive Vel Err", errR);
    
        //Tank Drive, but using fancy feedforwards stuff
        double l = calcDriveFF(targetVelL, limitAbsWithSign(errR * Constants.Drive.P + errSlopeL * Constants.Drive.D, Constants.Drive.MAX_ACC));
        double r = calcDriveFF(targetVelR, limitAbsWithSign(errL * Constants.Drive.P + errSlopeR * Constants.Drive.D, Constants.Drive.MAX_ACC));
    
        m_rightmotors.setVoltage(limitAbsWithSign(r, Constants.Drive.DRIVE_V_LIMIT));
        m_leftmotors.setVoltage (limitAbsWithSign(l, Constants.Drive.DRIVE_V_LIMIT));
        m_drive.feed();
    
        SmartDashboard.putNumber("Left Drive Output Voltage", l);
        SmartDashboard.putNumber("Right Drive Output Voltage",r);
    }

    public double SD_targetSpeedL = 0;
    public double SD_targetSpeedR = 0;
    public double SD_measuredSpeedL = 0;
    public double SD_measuredSpeedR = 0;
    public double SD_voltageL = 0;
    public double SD_voltageR = 0;

    public double getTSL() {return SD_targetSpeedL;};
    public double getTSR() {return SD_targetSpeedR;};
    public double getMSL() {return SD_measuredSpeedL;};
    public double getMSR() {return SD_measuredSpeedR;};
    public double getVL() {return SD_voltageL;};
    public double getVR() {return SD_voltageR;};

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Drivetrain :D");
        builder.addDoubleProperty("L Target Speed",   this::getTSL, x -> {});
        builder.addDoubleProperty("L Motor Voltage",  this::getVL,  x -> {});
        builder.addDoubleProperty("L Measured Speed", this::getMSL, x -> {});

        builder.addDoubleProperty("R Target Speed",   this::getTSR, x -> {});
        builder.addDoubleProperty("R Motor Voltage",  this::getVR,  x -> {});
        builder.addDoubleProperty("R Measured Speed", this::getMSR, x -> {});
    }
}
