package com.team1323.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;
import com.team1323.frc2017.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hanger extends Subsystem{
	private CANTalon motor1, motor2;
	public CANTalon getTalon(){
		return motor1;
	}
	
	private static Hanger instance = new Hanger();
	public static Hanger getInstance(){
		return instance;
	}
	public Hanger(){
		motor1 = new CANTalon(Ports.HANGER_1);
		motor2 = new CANTalon(Ports.HANGER_2);
		
		motor1.setCurrentLimit(40);
		motor1.EnableCurrentLimit(true);
		motor2.setCurrentLimit(40);
		motor2.EnableCurrentLimit(true);
		
		motor1.enableBrakeMode(true);
		motor2.enableBrakeMode(true);
		
		motor2.setStatusFrameRateMs(StatusFrameRate.Feedback, 1000);
		
		motor1.changeControlMode(TalonControlMode.PercentVbus);
		motor2.changeControlMode(TalonControlMode.PercentVbus);
	}
	
	public void hang(){
		motor1.set(1.0);
		motor2.set(1.0);
	}
	
	@Override
	public synchronized void stop(){
		motor1.set(0);
		motor2.set(0);
	}
	
	@Override
	public synchronized void zeroSensors(){
		//no-op
	}
	
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Hanger 1 Current", motor1.getOutputCurrent());
		//SmartDashboard.putNumber("Hanger 2 Current", motor2.getOutputCurrent());
		SmartDashboard.putNumber("Hanger 1 Voltage", motor1.getOutputVoltage());
		SmartDashboard.putNumber("Hanger 2 Voltage", motor2.getOutputVoltage());
	}
}
