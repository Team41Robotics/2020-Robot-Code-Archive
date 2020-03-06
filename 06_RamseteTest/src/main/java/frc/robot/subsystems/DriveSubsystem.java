/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private WPI_TalonSRX talonRF = new WPI_TalonSRX(DriveConstants.kRightMotor2Port);
  private WPI_TalonSRX talonRB = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
  private WPI_TalonSRX talonLF = new WPI_TalonSRX(DriveConstants.kLeftMotor2Port);
  private WPI_TalonSRX talonLB = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(talonLF, talonLB);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(talonRF, talonRB);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The gyro sensor
  public final AHRS navx = new AHRS(SerialPort.Port.kUSB1);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  Timer timer;
  private double prevTime = 0;
  private Pose2d prevPose = null;

  private NetworkTable odom;
  private NetworkTableEntry odomX, odomY, odomAngle, odomVelX, odomVelY, odomVelAngle;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    talonRB.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    talonLB.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  
    talonLB.setSensorPhase(true);
    navx.reset();

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    odom = NetworkTableInstance.getDefault().getTable("odometry");
    odomX = odom.getEntry("odomX");
    odomX.setDefaultDouble(0.0);
    odomY = odom.getEntry("odomY");
    odomY.setDefaultDouble(0.0);
    odomAngle = odom.getEntry("odomAngle");
    odomAngle.setDefaultDouble(0.0);
    odomVelX = odom.getEntry("odomVelX");
    odomVelX.setDefaultDouble(0.0);
    odomVelY = odom.getEntry("odomVelY");
    odomVelY.setDefaultDouble(0.0);
    odomVelAngle = odom.getEntry("odomVelAngle");
    odomVelAngle.setDefaultDouble(0.0);

    timer = new Timer();
    timer.start();
  }

  public double calcDistance(double encCounts) {
    
    return encCounts * DriveConstants.kEncoderDistancePerPulse;
  } 

  public double calcVelocity(double encRate) {

    return encRate* DriveConstants.kEncoderDistancePerPulse;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), calcDistance(talonLB.getSelectedSensorPosition()),
    calcDistance(talonRB.getSelectedSensorPosition()));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    Pose2d pose = m_odometry.getPoseMeters();
    
    System.out.println(pose);

    // Publish over network tables
    odomAngle.setDouble(pose.getRotation().getRadians());
    odomX.setDouble(pose.getTranslation().getX());
    odomY.setDouble(pose.getTranslation().getY());

    double currTime = timer.get();

    if(prevPose != null){

      //Calculate x and y component velocity
      double dx = pose.getTranslation().getX() - prevPose.getTranslation().getX();
      double dy = pose.getTranslation().getY() - prevPose.getTranslation().getY();
      //double dAngle = pose.getRotation().getRadians() - prevPose.getRotation().getRadians();
      double dt = currTime - prevTime;

      odomVelX.setDouble(dx/dt);
      odomVelY.setDouble(dy/dt);
      //odomVelAngle.setDouble(dAngle/dt);
    }

    prevPose = pose;
    prevTime = currTime;

    return pose;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(calcVelocity(talonLB.getSelectedSensorVelocity()), calcVelocity(talonRB.getSelectedSensorVelocity()));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    System.out.println(m_odometry.getPoseMeters());
    System.out.println(getAverageEncoderDistance());
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {

    talonRB.setSelectedSensorPosition(0);
    talonLB.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (calcDistance(talonLB.getSelectedSensorPosition()) + calcDistance(talonRB.getSelectedSensorPosition())) / 2.0;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    //return 0;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //return 0;
    return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
