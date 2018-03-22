package org.usfirst.frc.team619.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;

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
		if(intakeRight.getOutputCurrent() > 100)
		{
			intakeLeft.set(ControlMode.PercentOutput, -speed/2);
			intakeRight.set(ControlMode.PercentOutput, speed);
		}
		else if(intakeLeft.getOutputCurrent() > 100)
		{
			intakeRight.set(ControlMode.PercentOutput, -speed/2);
			intakeLeft.set(ControlMode.PercentOutput, speed);
		}
		else
		{
			intakeRight.set(ControlMode.PercentOutput, speed);
			intakeLeft.set(ControlMode.PercentOutput, speed);
		}
		
	}
	
	public void moveIntake(Hand hand, double speed)
	{
		if(hand.equals(Hand.kRight))
		{
			intakeRight.set(ControlMode.PercentOutput, speed);
		}
		if(hand.equals(Hand.kLeft))
		{
			intakeLeft.set(ControlMode.PercentOutput, speed);
		}
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
