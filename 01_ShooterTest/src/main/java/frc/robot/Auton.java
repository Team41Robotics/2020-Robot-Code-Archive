package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;

public class Auton {
    
	private PIDController distancePID, turnPID;
	private final double DISTANCE_Kp = 1, DISTANCE_Kd = 0, DISTANCE_Ki = 0;
    private final double TURN_Kp = 1, TURN_Kd = 0, TURN_Ki = 0;	
    private final double TURN_TOLERANCE = .1;

    private enum State {NONE, FIRST_TURN, FORWARD, SECOND_TURN};
    private State state = State.NONE;

    private Realsense realsense;
    private Driving drive;

    public Auton(Realsense realsense, Driving drive) {

        this.realsense = realsense;
        this.drive = drive;
    }

    //Runs to the realsense's set destination
	public void runToPoint() {		
		if(state == State.NONE) {
            state = State.FIRST_TURN;
        }
        else if(state == State.FIRST_TURN) {
            double angle = realsense.getAngleToDestination();
            if(angle - TURN_TOLERANCE < realsense.getCurrentAngle() && realsense.getCurrentAngle() < angle + TURN_TOLERANCE)  {
                state = State.FORWARD; //Switch to 2nd phase
            } else {
                drive.driveSpeed(-.5, .5); //Change to PID later
            }
        }
        else if(state == State.FORWARD) {
            double distance = realsense.getDistanceToDestination(); //Change this later to correct for drift
            if(realsense.getDistanceTraveled() < distance) {
                drive.driveSpeed(.3, .3); //Change to PID later
            } else {
                state = State.SECOND_TURN; //Switch to phase 3
            }
        }
        else if(state == State.SECOND_TURN) {
             double angle = realsense.getAngleFinal();
            if(angle - TURN_TOLERANCE < realsense.getCurrentAngle() && realsense.getCurrentAngle() < angle + TURN_TOLERANCE)  {
                state = State.NONE; //Finished!
            } else {
                drive.driveSpeed(-.5, .5); //Change to PID later
            }
        }
	}
}