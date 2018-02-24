package org.usfirst.frc.team619.robot;

public class AutoThread extends RobotThread{
	
	SwerveDriveBase driveBase;
	
	private WheelDrive backRight;
	private WheelDrive backLeft;
	private WheelDrive frontRight;
	private WheelDrive frontLeft;
	
	public AutoThread(int period, ThreadManager threadManager, WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft)
	{
		super(period, threadManager);
		
		this.backRight = backRight;
		this.backLeft = backLeft;
		this.frontRight = frontRight;
		this.frontLeft = frontLeft;
		
		driveBase = new SwerveDriveBase(backRight, backLeft, frontRight, frontLeft);
	}
	
	protected void cycle() 
	{
		
	}
	
}
