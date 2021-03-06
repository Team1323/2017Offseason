package com.team1323.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;
import com.team1323.frc2017.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallIntake extends Subsystem{
	private CANTalon intakeMotor;
	
	public boolean isIntaking = false;
	
	private static BallIntake instance = new BallIntake();
	public static BallIntake getInstance(){
		return instance;
	}
	
	public BallIntake(){
		intakeMotor = new CANTalon(Ports.BALL_INTAKE);
		intakeMotor.setCurrentLimit(30);
		intakeMotor.EnableCurrentLimit(true);
		intakeMotor.enableBrakeMode(false);
		intakeMotor.setStatusFrameRateMs(StatusFrameRate.Feedback, 1000);
		intakeMotor.changeControlMode(TalonControlMode.PercentVbus);
	}
	
	public void forward(){
		intakeMotor.set(1.0);
		isIntaking = true;
	}
	
	public void reverse(){
		intakeMotor.set(-1.0);
	}
	
	public void toggleForward(){
		if(isIntaking){
			stop();
		}else{
			forward();
		}
	}
	
	@Override
	public synchronized void stop(){
		intakeMotor.set(0);
		isIntaking = false;
	}
	
	@Override
	public synchronized void zeroSensors(){
		//no-op
	}
	
	@Override
	public void outputToSmartDashboard(){
		/*SmartDashboard.putNumber("Ball Intake Voltage", intakeMotor.getOutputVoltage());
		SmartDashboard.putNumber("Ball Intake Current", intakeMotor.getOutputCurrent());*/
	}
}
