/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team619.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	ThreadManager threadManager;
	TeleopThread swerveThread;
	
	AutoThread autoThread;
	
	//talons
	//drive id: 4  turn id: 9
	private WheelDrive backRight = new WheelDrive(4, 5);
	
	//drive id: 7  turn id: 8
	private WheelDrive backLeft = new WheelDrive(0, 1);
	
	//drive id: 6  turn id: 1
	private WheelDrive frontRight = new WheelDrive(6, 7);
	
	//drive id: 10  turn id: 3
	private WheelDrive frontLeft = new WheelDrive(2, 3);
	
	WheelDrive[] wheels = {backRight, backLeft, frontRight, frontLeft};
	
	Lift lift;
	TalonSRX lift1, lift2, lift3;
	
	Intake intake;
	TalonSRX intakeRight, intakeLeft;
	
	Ramp ramp;
	TalonSRX rampRelease;
	
	//Limit switches
	LimitSwitch intakeSwitch;
	
	//auto switches
	LimitSwitch[] autoSwitches = new LimitSwitch[4];
	
	AnalogUltrasonic[] ultrasonics = new AnalogUltrasonic[2];
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		intakeSwitch = new LimitSwitch(0);
		for(int i = 1; i <= 4; i++)
		{
			LimitSwitch autoSwitch = new LimitSwitch(i);
			autoSwitches[i-1] = autoSwitch;
		}
		
		ultrasonics[0] = new AnalogUltrasonic(0);
		ultrasonics[1] = new AnalogUltrasonic(2);
		
//		rampRelease = new TalonSRX(9);
//		
//		currentLimit(rampRelease, 35, 40);
//		
//		ramp = new Ramp(rampRelease);
//		
		lift1 = new TalonSRX(11);
		lift2 = new TalonSRX(12);
		lift3 = new TalonSRX(13);
			
		currentLimit(lift1, 20, 25);
		currentLimit(lift2, 20, 25);
		currentLimit(lift3, 20, 25);
		
		lift = new Lift(lift1, lift2, lift3);
		
		intakeLeft = new TalonSRX(10);
		intakeRight = new TalonSRX(14);
		
		currentLimit(intakeLeft, 25, 30);
		currentLimit(intakeRight, 25, 30);
		
		intake = new Intake(intakeLeft, intakeRight, intakeSwitch);
		
		threadManager = new ThreadManager();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		threadManager.killAllThreads();
		
		autoThread = new AutoThread(0, threadManager, backRight, backLeft, frontRight, frontLeft, lift, intake, autoSwitches, ultrasonics);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		//
	}

	private Joystick joystick = new Joystick(0);
	
	
	@Override
	public void teleopInit()
	{
		threadManager.killAllThreads();
		
		swerveThread = new TeleopThread(0, threadManager, backRight, backLeft, frontRight, frontLeft, lift, intake, ramp);
	}
	  
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		//swerveDrive.drive(joystick.getRawAxis(1), joystick.getRawAxis(0), joystick.getRawAxis(4));
		//System.out.println("CURRENT ANGLE: " + frontRight.getCurrentAngle());
		//System.out.println("COMMAND POSITION: " + frontRight.getTargetAngle());
//		System.out.println("RR: " + backRight.getRotateTalon().getSelectedSensorPosition(0) + " " + backRight.getCurrentAngle());
//		System.out.println("RL: " + backLeft.getRotateTalon().getSelectedSensorPosition(0) + " " + backLeft.getCurrentAngle());
//		System.out.println("FL: " + frontLeft.getRotateTalon().getSelectedSensorPosition(0) + " " + frontLeft.getCurrentAngle());
//		System.out.println("FR: " + frontRight.getRotateTalon().getSelectedSensorPosition(0) + " " + frontRight.getCurrentAngle());
//		System.out.println();
	}

	/**
	 * This function is called initially during test mode.
	 */
	@Override
	public void testInit() {
		threadManager.killAllThreads();
		//swerveThread = new SwerveThread(0, threadManager, backRight, backLeft, frontRight, frontLeft);
		
		System.out.println("testInit");
//		testDrive(backRight, 1);
//		testDrive(backLeft, 1);
//		testDrive(frontRight, 1);
//		testDrive(frontLeft, 1);
//		testRotate(frontLeft);
//		testRotate(frontRight);
//		testRotate(backRight);
//		testRotate(backLeft);
//		calibrate(backRight);
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		System.out.println("ultra0: " + ultrasonics[0].getDistanceIn());
		System.out.println("ultra1: " + ultrasonics[1].getDistanceIn());
		//System.out.println(backRight.getCurrentAngle() + " == " + backRight.getTargetAngle());
	}
	
	@Override
	public void disabledInit()
	{
		System.out.println("YOINK");
		threadManager.killAllThreads();
	}
	
    //DEBUG FUNCTIONS
    public void testDrive(WheelDrive wheel, double speed)
    {
    	wheel.setDriveSpeed(speed);
    	wheel.drive();
		delay(1000);
		wheel.stop();
		delay(1000);
		wheel.setDriveSpeed(-speed);
		wheel.drive();
		delay(1000);
		wheel.stop();
    }
	
    public void testRotate(WheelDrive wheel)
    {
//    	System.out.println("wheel current angle: " + wheel.getCurrentAngle());
//    	wheel.setTargetAngle(90);
//    	wheel.goToAngle();
//    	System.out.println("goto: " + wheel.getTargetAngle());
//    	delay(5000);
//    	System.out.println();
//    	System.out.println("wheel current angle: " + wheel.getCurrentAngle());
    	wheel.setTargetAngle(90);
    	wheel.goToAngle();
    	System.out.println("goto: " + wheel.getTargetAngle());
    	delay(3000);
    }
    
    public void calibrate(WheelDrive wheel)
    {
    	wheel.getRotateTalon().set(ControlMode.PercentOutput, 1);
    	delay(5000);
    	wheel.getRotateTalon().set(ControlMode.PercentOutput, 0);
    	delay(1000);
    	wheel.getRotateTalon().set(ControlMode.PercentOutput, -1);
    	delay(5000);
    	wheel.getRotateTalon().set(ControlMode.PercentOutput, 0);
    }
    
    public void delay(int milliseconds){
    	try {
    		Thread.sleep(milliseconds);
    	} catch(InterruptedException e) {
    		Thread.currentThread().interrupt();
    	}
    }
    
	/**
	 * Limits current to selected motor
	 * @param talon - TalonSRX object
	 * @param continousLimit - variable limit of current
	 * @param currentLimit - variable limit of peak
	 */
	public void currentLimit(TalonSRX talon, int continuousLimit, int currentLimit) {
		talon.configContinuousCurrentLimit(continuousLimit, 0);
		talon.configPeakCurrentLimit(currentLimit, 0);
		talon.configPeakCurrentDuration(100, 0);
		talon.enableCurrentLimit(true);

	}
    
}
