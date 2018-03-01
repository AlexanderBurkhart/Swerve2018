package org.usfirst.frc.team619.robot;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 *
 * This is the object for an ultrasonic sensor if you have it wired up
 * for analog only. (Pin 3 for analog and pins 6 and 7 for power on the sensor)
 * 
 * This sensor plugs into the Analog Breakout board on a CRio (not the NiRio)
 * 
 * This object was made on 11/9/14, roughly a month before the newer NiRio comes
 * out.
 * 
 * On the NiRio (also called Athena) the ulatrasonic sensor plugs into the analog section.
 * 
 * @author Chandler
 */
public class AnalogUltrasonic{
    
    private final AnalogInput analog;
    
    private static final double PULSE_TIME_SECONDS = 0.000020;
    private static final double INCHES_PER_VOLT = 253.238;
    private static final double CM_PER_VOLT = 99.7;
    private static final double MM_PER_VOLT = 997.0;
    
    public AnalogUltrasonic(int channel){
        analog = new AnalogInput(channel);
    }//end constructor
    
    public double get() {
        return analog.getVoltage();
    }
    
    public double getVoltage() {
        return analog.getVoltage();
    }
    
    public int getChannel() {
        return analog.getChannel();
    }
    
    public double getDistanceIn() {
        return analog.getVoltage() * INCHES_PER_VOLT;
    }

    //used for when you don't want relatively far objects to mess up readings i.e. a bit more accurate
    public double getDistanceIn(double limit) {
    	
    	double in = analog.getVoltage() * INCHES_PER_VOLT;
    	
    	if(in > limit)
    		return limit;
    	else
    		return in;
    	
    }
    
    public double getDistanceCM() {
        return analog.getVoltage() * CM_PER_VOLT;
    }

    public double getDistanceCM(double limit) {
    	
    	double cm = analog.getVoltage() * CM_PER_VOLT;
    	
    	if(cm > limit)
    		return limit;
    	else
    		return cm;
    	
    }
    
    public double getDistanceMM() {
        return analog.getVoltage() * MM_PER_VOLT;
    }
    
    public double getDistanceMM(double limit) {
    	
    	double mm = analog.getVoltage() * MM_PER_VOLT;
    	
    	if(mm > limit)
    		return limit;
    	else
    		return mm;
    	
    }
    
}//end object AnalogUltrasonic
