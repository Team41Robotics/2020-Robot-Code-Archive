package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;

public class Auton {
    
	private PIDController distancePID, turnPID;
	private final double DISTANCE_Kp = 1, DISTANCE_Kd = 0, DISTANCE_Ki = 0;
    private final double TURN_Kp = 1, TURN_Kd = 0, TURN_Ki = 0;	
    private final double TURN_TOLERANCE = .04;

    public enum State {NONE, FIRST_TURN, FORWARD, SECOND_TURN, END};
    private State state = State.NONE;

    private Realsense realsense;
    private Driving drive;
    private double speed = .5;

    public Auton(Realsense realsense, Driving drive) {

        this.realsense = realsense;
        this.drive = drive;
    }

    //Runs to the realsense's set destination
    /*
    Positive and less than PI -> turn Counterclock
    Positive and greater than PI -> turn clockwise
    Negative and abs value less than PI -> turn clockwise
    Negative and abs value more than PI -> turn counterclockwise
    */

    public void setState(State state) {
        this.state = state;
    }

    public void runToPoint() {		
        
		if(state == State.NONE) {
            state = State.FIRST_TURN;
        }
        else if(state == State.FIRST_TURN) {
            double angle = realsense.getAbsoluteAngleToDestination();
            double currentAngle = realsense.getCurrentAngle();
            if(angle - TURN_TOLERANCE < currentAngle && currentAngle < angle + TURN_TOLERANCE)  {
                state = State.FORWARD; //Switch to 2nd phase
            } else {
                if((angle - currentAngle > 0 && angle-currentAngle < Math.PI) || (angle - currentAngle < 0 && Math.abs(angle-currentAngle) > Math.PI))
                    drive.driveSpeed(-speed, speed); //Change to PID later
                else drive.driveSpeed(speed, -speed);
            }
            System.out.println("Destination angle: " + angle + " Current angle: " + realsense.getCurrentAngle());
        }
        else if(state == State.FORWARD) {
            double distance = realsense.getDistanceToDestination(); //Change this later to correct for drift
            if(realsense.getDistanceTraveled() < distance) {
                drive.driveSpeed(speed, speed); //Change to PID later
            } else {
                state = State.SECOND_TURN; //Switch to phase 3
            }
            System.out.println("Distance:" + distance + " traveled:" + realsense.getDistanceTraveled());
        }
        else if(state == State.SECOND_TURN) {
             double angle = realsense.getAngleFinal();
             double currentAngle = realsense.getCurrentAngle();
            if(angle - TURN_TOLERANCE < currentAngle && currentAngle < angle + TURN_TOLERANCE)  {
                state = State.END; //Finished!
            } else {
                if((angle - currentAngle > 0 && angle-currentAngle < Math.PI) || (angle - currentAngle < 0 && Math.abs(angle-currentAngle) > Math.PI))
                drive.driveSpeed(-speed, speed); //Change to PID later
            else drive.driveSpeed(speed, -speed);
                System.out.println("Destination angle: " + angle + " Current angle: " + realsense.getCurrentAngle());
            }
        } else {
            drive.driveSpeed(0, 0);
        }
	}
}