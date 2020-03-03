package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	private final double kP = .1, kI = 0.0000, kD = 0; //1.5
	private final int TIMEOUT_MS = 30;

	public enum DriveState {AUX_STRAIGHT, AUX_TURN, DRIVE};
	private DriveState driveState = DriveState.DRIVE;

	public Driving() {
		talonRF = new WPI_TalonSRX(PORTS.TALON_RF);
		talonRB = new WPI_TalonSRX(PORTS.TALON_RB);
		talonLF = new WPI_TalonSRX(PORTS.TALON_LF);
		talonLB = new WPI_TalonSRX(PORTS.TALON_LB);

		/**
		 * Auxiliary closed loop driving control using mag encoder values. This will be used to make the robot
		 * drive straight and do perfect point turns by matching the encoder values on each side. See:
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#auxpid-label
		 * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/DriveStraight_AuxQuadrature/src/main/java/frc/robot/Robot.java
		 */
		talonLB.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TIMEOUT_MS);
		
		// Inverts left side encoder direction to be in phase with left side motors
		talonLB.setSensorPhase(true);

		// Selects the mag encoder on the left side to be a remote sensor source for the right side
		talonRB.configRemoteFeedbackFilter(talonLB.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, 0, TIMEOUT_MS);
		
		// Configures a sensor term to be the difference of the encoders. This will be used for driving straight.
		talonRB.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, TIMEOUT_MS);
		talonRB.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, TIMEOUT_MS);

		// Configures a sensor term to be the sum of the encoders. This will be used for point turns.
		talonRB.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, TIMEOUT_MS);
		talonRB.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, TIMEOUT_MS);	
	
		// Configures auxiliary PID parameters
		talonRB.config_kP(1, kP, TIMEOUT_MS);
		talonRB.config_kI(1, kI, TIMEOUT_MS);
		talonRB.config_kD(1, kD, TIMEOUT_MS);

		talonRB.configAuxPIDPolarity(false, TIMEOUT_MS);

		// Selects profile slot 1 for auxiliary closed loop
		talonRB.selectProfileSlot(1, 1);

		resetEncoders();

		leftSpeedCG = new SpeedControllerGroup(talonLF, talonLB);
		rightSpeedCG = new SpeedControllerGroup(talonRF, talonRB);
		
		difDrive = new DifferentialDrive(leftSpeedCG, rightSpeedCG);
	}

		// Resets motor encoder values to 0
		public void resetEncoders() {

			talonRB.setSelectedSensorPosition(0);
			talonLB.setSelectedSensorPosition(0);
		}
	
		/**
		 * For some reason, using the auxiliary PID flips the polarity of the right side motors and the
		 * right encoder. The driveState keeps track of whether motors and encoders are configured for normal
		 * driving or aux PID, and setState() configures it accordingly. Look for a better fix, since its annoying. 
		 */
		public void setState(DriveState state) {
	
			switch(state) {
				case AUX_STRAIGHT:
					// Inverts the direction of the right encoder
					talonRB.setSensorPhase(true);
					
					// Inverts right side motor polarity
					talonRB.setInverted(true);
					talonRF.setInverted(true);
	
					// Configures auxiliarly PID to use the difference of the left and right sensors. Since the robot is
					// moving straight, the two encoder values should match, so the ideal setpoint is zero.
					talonRB.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, 1, TIMEOUT_MS);
					talonRB.configAuxPIDPolarity(false, TIMEOUT_MS);

					resetEncoders();
				break;
	
				case AUX_TURN:
					talonRB.setSensorPhase(false);
	
					talonRB.setInverted(false);
					talonRF.setInverted(false);
	
					
					// Configures auxiliary PID to use the sum of the left and right encoders. Since the robot is doing
					// a point turn, one of the encoders should be in the negative direction, so the ideal setpoint is zero.
					talonRB.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 1, TIMEOUT_MS);
					talonRB.configAuxPIDPolarity(true, TIMEOUT_MS);

					resetEncoders();
				break;
	
				case DRIVE:
	
					talonRB.setSensorPhase(false);
					
					talonRB.setInverted(false);
					talonRF.setInverted(false);
				break;
			}
	
			driveState = state;
		}

	/**
	 * <a href="http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf">Source Link</a>
	 * @param linear Linear velocity to drive at (in m/s)
	 * @param angular Angular velocity to rotate at (in rad/s)
	 */
	public void driveVelocity(double linear, double angular) {
		final double wheelBaseRadius = 0.3419; // Have to find this !!!
		final double wheelRadius = 0.1524 / 2.0; // Have to find this !!!
		double turnRadius = linear/angular; //v = r*omega

		double velocityLeft = angular*(turnRadius + wheelBaseRadius/2)/wheelRadius;
		double velocityRight = angular*(turnRadius - wheelBaseRadius/2)/wheelRadius;

		double vl = talonLB.getSelectedSensorVelocity();
		double vr = talonRB.getSelectedSensorVelocity();

		double inc = 0.01;

		if(vl < velocityLeft) { //Replace this with PID later?
			leftSpeed += inc;
		} 
		else if (vl > velocityLeft)
			leftSpeed -= inc;
		
		if(vr < velocityRight) {
			rightSpeed += inc;
		} 
		else if(vr > velocityRight){
			rightSpeed -= -inc;
		}
		// if(vl < velocityLeft && leftSpeed < 1) { //Replace this with PID later?
		// 	leftSpeed += inc;
		// } 
		// else if (vl > velocityLeft && leftSpeed > -1)
		// 	leftSpeed -= inc;
		
		// if(vr < velocityRight && rightSpeed < 1) {
		// 	rightSpeed += inc;
		// } 
		// else if(vr > velocityRight && rightSpeed > -1)
		// 	rightSpeed -= -inc;

		difDrive.tankDrive(leftSpeed, rightSpeed);
	}

	public void controllerMove(Joystick controller) {
		double leftAxis = controller.getRawAxis(BUTTONS.GAMEPAD.LEFT_JOY_Y_AXIS);
		double rightAxis = controller.getRawAxis(BUTTONS.GAMEPAD.RIGHT_JOY_Y_AXIS);

		double speedMultiplier = 0.55;

		if(driveState != DriveState.DRIVE) {
			setState(DriveState.DRIVE);
		}
		// System.out.println("Left: " + (talonLB.getSelectedSensorPosition()) + " Right: " + talonRB.getSelectedSensorPosition());

		//System.out.println("leftAxis: " + -leftAxis*speedMultiplier + " rightAxis: " + -rightAxis*speedMultiplier);
		difDrive.tankDrive(-leftAxis*speedMultiplier, -rightAxis*speedMultiplier);

		SmartDashboard.putNumber("left velocity", talonLB.getSelectedSensorVelocity());
		SmartDashboard.putNumber("right velocity", talonRB.getSelectedSensorVelocity());
	}

	public void driveSpeed(double leftSpeed, double rightSpeed) {
		
		if(driveState != DriveState.DRIVE) {
			setState(DriveState.DRIVE);
		}
		
		//System.out.println("Left: " + (talonLB.getSelectedSensorPosition()) + " Right: " + talonRB.getSelectedSensorPosition());
		//System.out.println("LeftSpeed: " + leftSpeed + " RightSpeed: " + rightSpeed);
		difDrive.tankDrive(leftSpeed, rightSpeed);
	}

	// Use auxiliary PID to match left side encoder value and right side encoder value
	public void driveStraight() {

		if(driveState != DriveState.AUX_STRAIGHT) {
			setState(DriveState.AUX_STRAIGHT);
		}

		// System.out.println("Left: " + (talonLB.getSelectedSensorPosition()) + " Right: " + talonRB.getSelectedSensorPosition());
		
		/** 
		 * Sets the percent output of the right master motor.The setpoint of the auxiliary PID is greater
		 * than zero because the left encoder tends to be always a little higher. Increasing the setpoint
		 * increases the difference between the right minus left encoders. This is a jank fix, look for 
		 * another way to match encoder values better.
		 */
		talonRB.set(ControlMode.PercentOutput, .3, DemandType.AuxPID, 50);
	
		// Sets the other right motor to follow the percent output of the right master motor
		talonRF.follow(talonRB, FollowerType.PercentOutput);

		// Sets the two left motors to use the auxiliary PID output of the right master
		talonLB.follow(talonRB, FollowerType.AuxOutput1);
		talonLF.follow(talonRB, FollowerType.AuxOutput1);

		
	}

	public void pointTurn() {
		
		 if(driveState != DriveState.AUX_TURN) {
		 	setState(DriveState.AUX_TURN);
		 }
		
		//driveSpeed(-.5, .5);
		 //System.out.println("Left: " + (talonLB.getSelectedSensorPosition()) + " Right: " + talonRB.getSelectedSensorPosition());
		 talonRB.set(ControlMode.PercentOutput, .3, DemandType.AuxPID, 0);
				
		 talonRF.follow(talonRB, FollowerType.PercentOutput);
		 talonLB.follow(talonRB, FollowerType.AuxOutput1);
		 talonLF.follow(talonRB, FollowerType.AuxOutput1);
	}
}