package com.team1323.frc2017.subsystems;

import com.ctre.PigeonImu;
import com.ctre.PigeonImu.PigeonState;
import com.team1323.frc2017.loops.Loop;
import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon {
	private PigeonImu pidgey;
	private Hanger hanger;
    private double currentAngle = 0.0;
    boolean pidgeonIsGood = false;
    double currentAngularRate = 0.0;
    PigeonImu.FusionStatus fusionStatus = new PigeonImu.FusionStatus();
    
	public Pigeon(){
		hanger = Hanger.getInstance();
		try{
			pidgey = new PigeonImu(hanger.getTalon());
			pidgey.EnableTemperatureCompensation(true);
		}catch(Exception e){
			System.out.println(e);
		}
	}
	private static Pigeon instance = null;
	public static Pigeon getInstance(){
		if(instance == null){
			instance = new Pigeon();
		}
		return instance;
	}
	public void update(){
		try{
			//PigeonImu.GeneralStatus genStatus = new PigeonImu.GeneralStatus();
			//PigeonImu.FusionStatus fusionStatus = new PigeonImu.FusionStatus();
			//double [] xyz_dps = new double [3];
			currentAngle = pidgey.GetFusedHeading(fusionStatus);
			//pidgey.GetGeneralStatus(genStatus);
			//pidgey.GetRawGyro(xyz_dps);
			pidgeonIsGood = (pidgey.GetState() == PigeonState.Ready) ? true : false;
			/*currentAngularRate = -xyz_dps[2];
			
			short [] ba_xyz = new short [3];
			pidgey.GetBiasedAccelerometer(ba_xyz);
			//SmartDashboard.putNumber("AccX", ba_xyz[0]);
			//SmartDashboard.putNumber("AccY", ba_xyz[1]);
			//SmartDashboard.putNumber("AccZ", ba_xyz[2]);
			
			double [] ypr = new double [3];
			pidgey.GetYawPitchRoll(ypr);
			//currentAngle = -ypr[0];*/
			
		}catch(Exception e){
			System.out.println(e);
		}
	}
	private final Loop pigeonLoop = new Loop(){
		@Override
		public void onStart(double timestamp){
			
		}
		@Override
		public void onLoop(double timestamp){
			update();
		}
		@Override
		public void onStop(double timestamp){
			
		}
	};
	public Loop getLoop(){
		return pigeonLoop;
	}
	public boolean isGood(){
		return pidgeonIsGood;
	}
	public double getAngle(){
		return currentAngle;
	}
	public double getAngularRate(){
		return currentAngularRate;
	}
	public void setAngle(double i){
		pidgey.SetFusedHeading(i);
		pidgey.SetYaw(i);
		System.out.println("Pigeon Angle Set");
	}
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber(" Heading Angle ", Util.boundAngleNeg180to180Degrees(getAngle()));
		SmartDashboard.putNumber(" Pigeon Rate ", getAngularRate());
		SmartDashboard.putBoolean(" Pigeon Good ", isGood());
		//SmartDashboard.putNumber("Pigeon Temp", pidgey.GetTemp());
	}
}
