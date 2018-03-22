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
	
	boolean turn;
	
	//0 = drive
	//1 = rotate
	//2 = drop
	boolean[] states = {false, false, false};
	boolean dropped;
	
	double startTime;
	double targetTime;
	boolean moving;
	boolean intaking;
	int targetAngle;
	
	//drive vars
	double x1;
	double y1;
	double x2;
	double s1;
	
	double turnVal;
	
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
		s1 = 0;
		
		imu = new AHRS(SPI.Port.kMXP);
		
		driveBase = new SwerveDriveBase(backRight, backLeft, frontRight, frontLeft);
		
		gameData =  DriverStation.getInstance().getGameSpecificMessage();
		
		//REMOVE WHEN DONE TESTING
		gameData = "RLR";
		
		side = Character.toString(gameData.charAt(0));
		
		configure();
		
		startAuto();
		start();
	}
	
	
	public void startAuto()
	{
		//left
		if(autoType[0] && side.equals("L"))
		{
			//move forward
			move(0, 0.3, 1000);
			//turn 90 clockwise
			turnTo(90);
			//intake
			intake(1, 1000);
		}
		else if(autoType[0] && side.equals("R"))
		{
			//move forward
			move(0, 0.3, 1000);
			//turn 180
			turnTo(180);
			//move right
			move(0.3, 0, 1000);
			//intake
			intake(1, 1000);
		}
		
		//right
		if(autoType[1] && side.equals("R"))
		{
			//move forward
			move(0, 0.3, 1000);
			//turn 90 counter clockwise
			turnTo(270);
			//intake
			intake(1, 1000);
		}
		else if(autoType[1] && side.equals("L"))
		{
			//move forward
			move(0, 0.3, 1000);
			//turn 180
			turnTo(180);
			//move right
			move(-0.3, 0, 1000);
			//intake
			intake(1, 1000);
		}
	}
	
	public void turnTo(int angle)
	{
		targetAngle = angle;
		turn = true;
	}
	
	public int getTargetAngle()
	{
		return targetAngle;
	}
	
	public void cycle()
	{
		System.out.println(imu.getAngle() + " = " + getTargetAngle() + " is " + !atAngle() + " turn = " + turn);
		if(turn)
		{
			if(!atAngle())
			{
				turnVal = side.equals("R") ? -0.3 : 0.3;
				//System.out.println("turn bb : " + turnVal);
			}
			else
			{
				//System.out.println("stop bb");
				turnVal = 0;
				turn = false;
			}
		}
		
		if(moving)
		{
			double currentTime = System.currentTimeMillis();
			
			double time = currentTime - startTime;
			
			if(time >= targetTime)
			{
				x1 = 0;
				y1 = 0;
				time = 0;
				moving = false;
			}
		}
		
		if(intaking)
		{
			double currentTime = System.currentTimeMillis();
			
			double time = currentTime - startTime;
			
			if(time >= targetTime)
			{
				s1 = 0;
				time = 0;
				intaking = false;
			}
		}
		
		intake.moveIntake(s1);
		driveBase.getFieldCentric(x1, y1, turnVal);
	}
	
	public void move(double x, double y, int time)
	{
		moving = true;
		
		x1 = x;
		y1 = y;
		
		startTime = System.currentTimeMillis();
		targetTime = time;
	}
	
	public void intake(double s, int time)
	{
		intaking = true;
		
		s1 = s;
		
		startTime = System.currentTimeMillis();
		targetTime = time;
	}
	
	public boolean atAngle()
	{
		if(imu.getAngle() > getTargetAngle()-10 && imu.getAngle() < getTargetAngle()+10)
		{
			return true;
		}
		return false;
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
		
		turnVal = 0;
		
		liftSetup();
	}
	
	
	
	public void liftSetup()
	{
		liftDown();
		moveInPosition();
	}
	
	public void liftDown()
	{
		//set lift down
		lift.moveLift(1);
		delay(300);
		lift.moveLift(-0.5);
		delay(400);
		lift.moveLift(0);
		delay(1000);
	}
	
	public void moveInPosition()
	{
		lift.moveLift(1);
		delay(1000);
		lift.moveLift(0);
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
}
