package org.usfirst.frc.team619.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake {
	
	TalonSRX intakeRight;
	TalonSRX intakeLeft;
	
	LimitSwitch intakeSwitch;
	
	public Intake(TalonSRX ir, TalonSRX il, LimitSwitch iSwitch)
	{
		intakeRight = ir;
		intakeLeft = il;
		intakeSwitch = iSwitch;
	}
	
	/**
	 * Controls speed and direction of intake motors
	 * @param speed
	 */
	public void moveIntake(double speed)
	{
		//- for intakeright is in
		//+ for intakeleft is in
		
		System.out.println(intakeSwitch.get());
		if(intakeSwitch.get() == true)
		{
			intakeRight.set(ControlMode.PercentOutput, speed/2);	
		}
		else
		{
			intakeRight.set(ControlMode.PercentOutput, -speed);
		}
		intakeLeft.set(ControlMode.PercentOutput, speed);
	}
	
	/**
	 * Stops intake
	 */
	public void stopIntake()
	{
		intakeRight.set(ControlMode.PercentOutput, 0);
		intakeLeft.set(ControlMode.PercentOutput, 0);
	}
}
