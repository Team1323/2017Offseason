package com.team1323.frc2017;

public class Constants {
	public static final double kLooperDt = 0.005;
	
	public static final double kDriveWheelDiameterInches = 4.0;
	public static final double kTrackWidthInches = 31.5;
    public static final double kTrackScrubFactor = 1.0;
	
	public static final double kShootingSpeed = 3100.0;
	public static final double kShooterAllowableError = 100.0;//rpm
	public static final double kOptimalShootingRange = 70.0;
	
 // Path following constants
	public static final double kSegmentCompletionTolerance = 0.1; // inches
	public static final double kPathFollowingMaxAccel = 120.0; // inches per second^2
    public static final double kPathFollowingMaxVel = 120.0; // inches per second
    
 // Goal tracker constants
    public static final double kMaxGoalTrackAge = 1.0;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 30.0;
    public static final double kTrackReportComparatorStablityWeight = 1.0;
    public static final double kTrackReportComparatorAgeWeight = 1.0;
    
 // Pose of the camera frame w.r.t. the robot frame
    public static final double kCameraXOffset = 0.0;
    public static final double kCameraYOffset = 0.0;
    public static final double kCameraZOffset = 20.9;//TBM
    public static final double kCameraPitchAngleDegrees = 29.56; //TBM
    public static final double kCameraYawAngleDegrees = 0.0;
    public static final double kCameraDeadband = 0.0;
    
    public static final double kBoilerTargetTopHeight = 88.0;
    public static final double kBoilerRadius = 7.5;
    
 // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 1.2;
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 144.0;
    public static double kDriveHighGearVelocityKf = 1023.0/5000.0;
    public static int kDriveHighGearVelocityIZone = 0;
    public static double kDriveHighGearVelocityRampRate = 240.0;
    public static double kDriveHighGearNominalOutput = 0.5;
    public static double kDriveHighGearMaxSetpoint = 14.0 * 12.0; // 17 fps
    
    public static final double kDriveLowGearPositionKp = 1.6;
    public static final double kDriveLowGearPositionKi = 0.003;
    public static final double kDriveLowGearPositionKd = 128.0;
    public static final double kDriveLowGearPositionKf = 1023.0/5000.0;
    public static final int kDriveLowGearPositionIZone = 50;
    public static final double kDriveLowGearPositionRampRate = 240.0; // V/s
    public static final double kDriveLowGearNominalOutput = 0.5; // V
    public static final double kDriveLowGearMaxVelocity = 10.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 6 fps
                                                                                                               // in RPM
    public static final double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
                                                                                                             // in RPM/s
    
 // Phone
    public static final int kAndroidAppTcpPort = 8254;
}
