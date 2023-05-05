package frc.robot;

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

import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.SerialPort.Port;

import edu.wpi.first.wpilibj.SPI.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot {

	// ========================== ROBOT HARDWARE ==========================//

	private final CANSparkMax usedForGrabbingEncoderR = new CANSparkMax(21, MotorType.kBrushless);

	// TODO: Possibly refactor this with most of the spark maxes .follow()ing a
	// leader
	// Then again, might make things more annoying
	private final MotorController m_rightmotors = new MotorControllerGroup(
			usedForGrabbingEncoderR,
			new CANSparkMax(22, MotorType.kBrushless),
			new CANSparkMax(23, MotorType.kBrushless));

	private final CANSparkMax usedForGrabbingEncoderL = new CANSparkMax(24, MotorType.kBrushless);
	private final MotorController m_leftmotors = new MotorControllerGroup(
			usedForGrabbingEncoderL,
			new CANSparkMax(25, MotorType.kBrushless),
			new CANSparkMax(26, MotorType.kBrushless));

	private final DifferentialDrive m_drive = new DifferentialDrive(m_leftmotors, m_rightmotors);
	private final RelativeEncoder encoderL = usedForGrabbingEncoderL.getEncoder();
	private final RelativeEncoder encoderR = usedForGrabbingEncoderR.getEncoder();

	// private final AHRS navXIMU = new AHRS(Port.kUSB2); //USB2 is BOTTOM USB
	// PORT!!!
	private final AHRS navXIMU = new AHRS(Port.kMXP);

	private final Solenoid shiftinator = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

	public enum Gear {
		HIGH_GEAR,
		LOW_GEAR
	}

	public Gear gear = Gear.LOW_GEAR;

	public final Solenoid grabinator = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
	public final Solenoid liftinator = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

	public void init() {
		shiftinator.set(true); // Low gear is solenoid on

		m_leftmotors.setInverted(false); // TODO: Check that I'm actually going the right way
		m_rightmotors.setInverted(true);

	}

	/**
	 * Clamp value between +limit and -limit
	 * 
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

	// TODO: Consider whether or not to stop using custom feedforwards (keeping it
	// makes code easier to explain, but could have bugs)
	private double calcDriveFF(double targetVel, double targetAcc) {
		return Constants.Drive.KS * Math.signum(targetVel)
				+ Constants.Drive.KV * targetVel
				+ Constants.Drive.KA * targetAcc;
	}

	public void tankDriveWithFF(double targetVelL, double targetVelR, double currentVelL, double currentVelR) {

		SmartDashboard.putNumber("Left Drive Target Vel", targetVelL);
		SmartDashboard.putNumber("Right Drive Target Vel", targetVelR);

		double targetAccelerationL = targetVelL - currentVelL;
		double targetAccelerationR = targetVelR - currentVelR;

		// Tank Drive, but using fancy feedforwards stuff
		double l = calcDriveFF(targetVelL, limitAbsWithSign(targetAccelerationL, Constants.Drive.MAX_ACC));
		double r = calcDriveFF(targetVelR, limitAbsWithSign(targetAccelerationR, Constants.Drive.MAX_ACC));

		m_rightmotors.setVoltage(limitAbsWithSign(r, Constants.Drive.DRIVE_V_LIMIT));
		m_leftmotors.setVoltage(limitAbsWithSign(l, Constants.Drive.DRIVE_V_LIMIT));
		m_drive.feed();

		SmartDashboard.putNumber("Left Drive Output Voltage", l);
		SmartDashboard.putNumber("Right Drive Output Voltage", r);
	}

	// Stop all moving parts of the robot immediatly
	public void stopMovingNow() {
		m_drive.stopMotor();
	}

	public void setShifterTo(Gear gear) {
		switch (gear) {
			case HIGH_GEAR:
				shiftinator.set(false);
				break;
			case LOW_GEAR:
				shiftinator.set(true);
				break;
		}
	}

	public double getVelL() {
		return adjustByGearRatio(encoderL.getVelocity());
	}

	public double getVelR() {
		return adjustByGearRatio(encoderR.getVelocity());
	}

	public double getPosL() {
		return adjustByGearRatio(encoderL.getPosition());
	}

	public double getPosR() {
		return adjustByGearRatio(encoderR.getPosition());
	}

	private double adjustByGearRatio(double val) {
		switch (gear) {
			case LOW_GEAR:
				val *= Constants.Drive.LOW_GEAR;
			case HIGH_GEAR:
				val *= Constants.Drive.HIGH_GEAR;
		}
		return val;
	}

	public float getPitch() {
		return navXIMU.getPitch();
	}

	public float getYaw() {
		return navXIMU.getYaw();
	}

	public float getRoll() {
		return navXIMU.getRoll();
	}

}
