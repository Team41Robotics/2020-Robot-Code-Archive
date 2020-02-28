package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Driving {

	private WPI_TalonSRX talonRF, talonRB, talonLF, talonLB;

	private SpeedControllerGroup leftSpeedCG, rightSpeedCG;
	private DifferentialDrive difDrive;

	// For velocity drive
	private double leftSpeed = 0;
	private double rightSpeed = 0;

	// For auxiliary PID
	private final double kP = 1.5, kI = 0.0000, kD = 0;
	private final int TIMEOUT_MS = 30;

	public enum DriveState {AUX, DRIVE};
	private DriveState driveState = DriveState.DRIVE;

	public Driving() {
		talonRF = new WPI_TalonSRX(PORTS.TALON_RF);
		talonRB = new WPI_TalonSRX(PORTS.TALON_RB);
		talonLF = new WPI_TalonSRX(PORTS.TALON_LF);
		talonLB = new WPI_TalonSRX(PORTS.TALON_LB);

		// Auxiliary closed loop driving control using encoder values. Consider changing to velocity drive so we don't have to keep resetting encoder values.
		// https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#auxpid-label
		// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/DriveStraight_AuxQuadrature/src/main/java/frc/robot/Robot.java

		talonLB.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TIMEOUT_MS);
		
		// Inverts encoder direction
		talonLB.setSensorPhase(true);

		//talonRB.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		talonRB.configRemoteFeedbackFilter(talonLB.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, 0, TIMEOUT_MS);
		
		talonRB.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, TIMEOUT_MS);	// Feedback Device of Remote Talon
		talonRB.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, TIMEOUT_MS);

		talonRB.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, 1, TIMEOUT_MS);	
	
		// Configures auxiliary PID parameters
		talonRB.config_kP(1, kP, TIMEOUT_MS);
		talonRB.config_kI(1, kI, TIMEOUT_MS);
		talonRB.config_kD(1, kD, TIMEOUT_MS);

		talonRB.configAuxPIDPolarity(false, TIMEOUT_MS);
		talonRB.selectProfileSlot(1, 1);

		resetEncoders();

		leftSpeedCG = new SpeedControllerGroup(talonLF, talonLB);
		rightSpeedCG = new SpeedControllerGroup(talonRF, talonRB);

		difDrive = new DifferentialDrive(leftSpeedCG, rightSpeedCG);
	}

	/**
	 * <a href="http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf">Source Link</a>
	 * @param linear Linear velocity to drive at (in m/s)
	 * @param angular Angular velocity to rotate at (in rad/s)
	 */
	public void driveVelocity(double linear, double angular) {
		final double wheelBaseRadius = 0.5969; // Have to find this !!!
		final double wheelRadius = 0.1; // Have to find this !!!
		double turnRadius = linear/angular; //v = r*omega

		double velocityLeft = angular*(turnRadius + wheelBaseRadius/2)/wheelRadius;
		double velocityRight = angular*(turnRadius - wheelBaseRadius/2)/wheelRadius;

		double vl = talonLB.getSelectedSensorVelocity();
		double vr = talonRB.getSelectedSensorVelocity();

		if(vl < velocityLeft && leftSpeed < 1) { //Replace this with PID later?
			leftSpeed += .001;
		} 
		else if (vl > velocityLeft && leftSpeed > -1)
			leftSpeed -= .001;
		
		if(vr < velocityRight && rightSpeed < 1) {
			rightSpeed += .001;
		} 
		else if(vr > velocityRight && rightSpeed > -1)
			rightSpeed -= .001;

		difDrive.tankDrive(leftSpeed, rightSpeed);
	}

	public void controllerMove(Joystick controller) {
		double leftAxis = controller.getRawAxis(BUTTONS.GAMEPAD.LEFT_JOY_Y_AXIS);
		double rightAxis = controller.getRawAxis(BUTTONS.GAMEPAD.RIGHT_JOY_Y_AXIS);

		double speedMultiplier = 0.55;

		if(driveState != DriveState.DRIVE) {
			System.out.println("setDrive() Called from Controller");
			setDrive();
		}

		//System.out.println("Left: " + (talonLB.getSelectedSensorPosition()) + " Right: " + talonRB.getSelectedSensorPosition());
		//System.out.println("leftAxis: " + -leftAxis*speedMultiplier + " rightAxis: " + -rightAxis*speedMultiplier);
		difDrive.tankDrive(-leftAxis*speedMultiplier, -rightAxis*speedMultiplier);
		//if(leftAxis > 0) driveVelocity(0, .5); //.5 radians per second
	}

	public void driveSpeed(double leftSpeed, double rightSpeed) {
		
		if(driveState != DriveState.DRIVE) {
			System.out.println("setDrive() called from driveSpeed");
			setDrive();
		}
		
		//System.out.println("Left: " + (talonLB.getSelectedSensorPosition()) + " Right: " + talonRB.getSelectedSensorPosition());
		//System.out.println("LeftSpeed: " + leftSpeed + " RightSpeed: " + rightSpeed);
		difDrive.tankDrive(leftSpeed, rightSpeed);
	}

	// For some reason, using the auxiliary PID flips the polarity of the right side motors and the
	// right encoder. the driveState keeps track of whether motors and encoders are configured for
	// normal driving or PID, and setAux() or setDrive() configures it accordingly. Look for a better fix.
	public void setAux() {

		driveState = DriveState.AUX;

		// Inverts the direction of the right encoder
		talonRB.setSensorPhase(true);
		
		// Inverts right side motor polarity
		talonRB.setInverted(true);
		talonRF.setInverted(true);

		resetEncoders();
	}

	public void setDrive() {

		driveState = DriveState.DRIVE;

		talonRB.setSensorPhase(false);
		
		talonRB.setInverted(true);
		talonRF.setInverted(true);
	}

	// Resets motor encoder values to 0
	public void resetEncoders() {

		talonRB.setSelectedSensorPosition(0);
		talonLB.setSelectedSensorPosition(0);
	}

	// Use auxiliary PID to match left side encoder value and right side encoder value
	public void driveStraight() {

		if(driveState != DriveState.AUX) {
			setAux();
		}

		System.out.println("Left: " + (talonLB.getSelectedSensorPosition()) + " Right: " + talonRB.getSelectedSensorPosition());
		
		// Sets the percent output of the right master motor
		// The setpoint of the auxiliary PID is greater than zero because the left encoder tends to be always a little higher.
		// Increasing the setpoint increases the difference between the right minus left encoders.
		// This is a jank fix, look for another way to match encoder values better
		talonRB.set(ControlMode.PercentOutput, .2, DemandType.AuxPID, 50);
	
		// Sets the other right motor to follow the percent output of the right master motor
		talonRF.follow(talonRB, FollowerType.PercentOutput);

		// Sets the two left motors to use the auxiliary PID output of the right master
		talonLB.follow(talonRB, FollowerType.AuxOutput1);
		talonLF.follow(talonRB, FollowerType.AuxOutput1);
	}
}