package org.usfirst.frc.team619.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Ramp {
	
	TalonSRX ramp;
	
	public Ramp(TalonSRX r)
	{
		ramp = r;
	}
	
	public void windUp()
	{
		ramp.set(ControlMode.PercentOutput, 0.5);
	}
	
	public void windDown()
	{
		ramp.set(ControlMode.PercentOutput, -0.5);
	}
	
	public void stop()
	{
		ramp.set(ControlMode.PercentOutput, 0);
	}
	
}
