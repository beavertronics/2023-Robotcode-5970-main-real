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
//import frc.robot.Constants.LogitechF130Controller;

import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.first.cameraserver.CameraServer; //Not needed because jetson stuff is fancy

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController; //UNRIP Xbox controller

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Robot.Gear;

//=================Large TODOs:
//TODO: Refactor to use a command based architechture (Will make auto programming 10 times easier, but will take a long time)
//TODO: Properly sort out drive directions
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




public class RobotControl extends TimedRobot {

	//================== DRIVER STATION HARDWARE (controllers) ==================//

	private final XboxController joyOperator = new XboxController(0);
	private final Joystick joyL = new Joystick(1);
	private final Joystick joyR = new Joystick(2);

	private final Robot bot = new Robot();

	/*Auto Switching setup */
	enum Autos {
			LEAVE ("Just drive forwards to leave the Community"),
			SCORE_LEAVE ("Drive backwards to score, then drive forwards to leave the community."),
			SCORE_LEAVE_BALANCE ("Score, drive over platform to get mobility, then reverse and balance platform."),
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
		bot.init();
	
		autoChooser.setDefaultOption(Autos.NOTHING.desc, Autos.NOTHING);
		for (Autos auto : Autos.values()) {
			autoChooser.addOption(auto.desc, auto);
		}
		SmartDashboard.putData("Auto Choices:", autoChooser);


		//CameraServer.startAutomaticCapture(); //jetson stuff replaces this
		//TODO: Include jetson stuff in this repo??
	}

	/*Called every 20 ms, no matter the mode.*/
	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("NAVX Pitch",        bot.getPitch());
		SmartDashboard.putNumber("NAVX Roll",         bot.getRoll() );
		SmartDashboard.putNumber("NAVX Yaw(Heading)", bot.getYaw()  );
	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		System.out.println("Teleop Initialized!");
	}


	boolean dangermode = false;
	
	@Override
	public void teleopPeriodic() {
		if (joyOperator.getLeftBumper()) {
			bot.stopMovingNow();
			return;
		}

		//============TELEOP DRIVING!!
		double joyl = joyL.getY();
		double joyr = joyR.getY();

		if (Math.abs(joyl) < 0.06) joyl = 0;
		if (Math.abs(joyr) < 0.06) joyr = 0;

		if (joyOperator.getRightBumperPressed()) dangermode = !dangermode;

		if (joyL.getTrigger() && dangermode) {
			joyl *= Constants.Drive.TELE_FAST_SPEED_MULT;
			joyr *= Constants.Drive.TELE_FAST_SPEED_MULT;
		} 
		else {
			joyl *= Constants.Drive.TELE_NORM_SPEED_MULT;
			joyr *= Constants.Drive.TELE_NORM_SPEED_MULT;
		}

		bot.tankDriveWithFF(
			joyl, 
			joyr, 
			bot.getVelL(),
			bot.getVelR()
		);
		

		//Solonoids

		//==============SHIFTING!!
	
		if (dangermode) {
			if (joyR.getRawButton(2)) {
				bot.setShifterTo(Gear.HIGH_GEAR);
			} else {
				bot.setShifterTo(Gear.LOW_GEAR);
			}
		}       

		//==============GRABBY CLAW!!!!

		if (joyOperator.getXButton()) bot.grabinator.set(true);
		if (joyOperator.getYButton()) bot.grabinator.set(false);
		if (joyOperator.getAButton()) bot.liftinator.toggle();
		
	}

	@Override
	public void autonomousInit() {

	}

	@Override
	public void autonomousPeriodic() {
		
	}


	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		bot.stopMovingNow();
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {
		bot.stopMovingNow(); //Just in case ;)
	}

	@Override
	public void testInit() {
		System.out.println("Test mode Init!");
	}

	@Override
	public void testPeriodic() {}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {}
}
