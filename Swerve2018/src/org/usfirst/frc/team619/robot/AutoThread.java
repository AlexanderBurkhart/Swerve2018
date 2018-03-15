package org.usfirst.frc.team619.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class AutoThread extends RobotThread{
	
	SwerveDriveBase driveBase;
	
	private WheelDrive backRight;
	private WheelDrive backLeft;
	private WheelDrive frontRight;
	private WheelDrive frontLeft;
	
	LimitSwitch[] autoSwitches;
	AnalogUltrasonic[] ultrasonics;
	Lift lift;
	Intake intake;
	
	boolean[] autoType = {false, false, false, false};
	
	AHRS imu;
	
	String gameData;
	String side;
	
	//alternative to drive encoder atm
	int count = 0;
	double startingTime, currentTime;
	
	//0 = drive
	//1 = rotate
	//2 = drop
	boolean[] states = {false, false, false};
	boolean dropped;
	int targetAngle;
	public AutoThread(int period, ThreadManager threadManager, WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft, Lift l, Intake i, LimitSwitch[] aSwitches, AnalogUltrasonic[] anasonics)
	{
		super(period, threadManager);
		
		this.backRight = backRight;
		this.backLeft = backLeft;
		this.frontRight = frontRight;
		this.frontLeft = frontLeft;
		
		autoSwitches = aSwitches;
		ultrasonics = anasonics;
		
		startingTime = System.currentTimeMillis();
		
		dropped = false;
		
		lift = l;
		intake = i;
		
		imu = new AHRS(SPI.Port.kMXP);
		
		driveBase = new SwerveDriveBase(backRight, backLeft, frontRight, frontLeft);
		
		gameData =  DriverStation.getInstance().getGameSpecificMessage();
		
		//REMOVE WHEN DONE TESTING
		//gameData = "RLR";
		
		side = Character.toString(gameData.charAt(0));
		
		configure();
		start();
	}
	
	
	/**
	 * autoType config
	 * <p>
	 * 0 = left position
	 * <p>
	 * 1 = right position
	 * <p>
	 * 2 = scale auto
	 * <p>
	 * 3 = switch auto
	 */
	public void configure()
	{
		for(int i = 0; i < 4; i++)
		{
			if(autoSwitches[i].get() == true)
			{
				autoType[i] = true;
			}
		}
 
		//set lift down
		lift.moveLift(1);
		delay(300);
		lift.moveLift(-0.5);
		delay(400);
		lift.moveLift(0);
		delay(1000);
		
		//move lift in position
		moveInPosition();
	}
	
	public void moveInPosition()
	{
		lift.moveLift(1);
		delay(1000);
		lift.moveLift(0);
	}
	
	/**
	 * refreshes updates continuously
	 */
	protected void cycle() 
	{
		if(side.equals("L")) 
		{
			
			//if in left position
			if(autoSwitches[0].get() == true || autoSwitches[3].get() == true)
			{
				leftAuto(true);
			}
			//if in right position
			else if(autoSwitches[1].get() == true)
			{
				rightAuto(false);
			}
			else
			{
				moveForward();
			}
		}
		else if(side.equals("R"))
		{
			//if in left position
			if(autoSwitches[0].get() == true)
			{
				leftAuto(false);
			}
			//if in right position
			else if(autoSwitches[1].get() == true  || autoSwitches[4].get() == true)
			{
				rightAuto(true);
			}
			else
			{
				moveForward();
			}
		}
	}
	
	/**
	 * auto algorithm when in the left position
	 * @param sameSide - if in the same position as your own aliance's switch
	 */
	public void leftAuto(boolean sameSide)
	{
		//if on sameSide, drive forward to switch and drop cube
		if(sameSide)
		{
			//if not close to object keep driving
			if(!closeToObject() && !dropped)
			{
				driveBase.drive(0.25, 0, 0);
			}
			//else run intake as long as its the switch
			else
			{
				//checks for edge case
				delay(50);
				//if still close to object stop and run intake
				if(closeToObject() && !dropped)
				{
					driveBase.drive(0, 0, 0);
					intake.moveIntake(-0.5);
					delay(500);
					intake.moveIntake(0);
					dropped = true;
				}
			}
		}
		//if not on the same side, drive forward to switch, drive right and drop it into the other side
		else
		{
			//if not close to object keep driving
			if(!closeToObject() && !dropped)
			{
				driveBase.drive(0.25, 0, 0);
			}
			else
			{
				//chceks for edge case
				delay(50);
				//if still close to object drive sideways and drop cube
				if(closeToObject() && !dropped)
				{
					//TODO: set up PID with drive to go to encoderPosition
					driveBase.drive(0, 0.25, 0);
					delay(3000);
					driveBase.drive(0, 0, 0);
					intake.moveIntake(-0.5);
					delay(500);
					intake.moveIntake(0);
				}
			}
		}
		
	}
	
	/**
	 * auto algorithm when in the right position
	 * @param sameSide - if in the same position as your own aliance's switch
	 */
	public void rightAuto(boolean sameSide)
	{
		//if on sameSide, drive forward to switch and drop cube
		if(sameSide)
		{
			//if not close to object keep driving
			if(!closeToObject() && !dropped)
			{
				driveBase.drive(0.25, 0, 0);
			}
			//else run intake as long as its the switch
			else
			{
				//ultrasonics[0].getDistanceIn();
				//checks for edge case
				delay(50);
				//if still close to object stop and run intake
				if(closeToObject() && !dropped)
				{
					driveBase.drive(0, 0, 0);
					intake.moveIntake(-0.5);
					delay(500);
					intake.moveIntake(0);
					dropped = true;
				}
			}
		}
		//if not on the same side, drive forward to switch, drive left and drop it into the other side
		else
		{
			//if not close to object keep driving
			if(!closeToObject() && !dropped)
			{
				driveBase.drive(0.25, 0, 0);
			}
			//else prepare to drive sideways
			else
			{
				//chceks for edge case
				delay(50);
				//if still close to object drive sideways and drop cube
				if(closeToObject() && !dropped)
				{
					//TODO: set up PID with drive to go to encoderPosition
					driveBase.drive(0, -0.25, 0);
					delay(3000);
					driveBase.drive(0, 0, 0);
					intake.moveIntake(-0.5);
					delay(500);
					intake.moveIntake(0);
					dropped = true;
				}
			}
		}
	}
	
	public void moveForward()
	{
		currentTime = System.currentTimeMillis() - startingTime;
		if(currentTime < 3000)
		{
			driveBase.drive(0.25, 0, 0);
		}
		else
		{
			driveBase.drive(0, 0, 0);
		}
	}
	
	/*
	 * return true if both ultrasonics sees a close object
	 */
	public boolean closeToObject()
	{
		//return (ultrasonics[0].getDistanceIn() < 80 && ultrasonics[1].getDistanceIn() < 80);
		return (ultrasonics[0].getDistanceIn() < 125);
	}
	
	/**
	 * Delays thread in milliseconds
	 * @param milliseconds - variable time to delay in ms 
	 */
    public void delay(int milliseconds){
    	try {
    		Thread.sleep(milliseconds);
    	} catch(InterruptedException e) {
    		Thread.currentThread().interrupt();
    	}
    }
    
    public void turnToAngle(int targetAngle) 
    {
    	double theta = imu.getYaw( );
    	if(targetAngle-theta < 2) 
    	{
    		
    	}
    	else if(targetAngle-theta < -2)
    	{
    		
    	}
    }
}
