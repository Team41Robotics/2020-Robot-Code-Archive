package frc.robot;

public class Auton {
    
    private final double TURN_TOLERANCE = .04;

    public enum State {BEGIN, FIRST_TURN, FORWARD, SECOND_TURN, END};
    private State state = State.BEGIN;

    private Realsense realsense;
    private Driving drive;
    private double speed = .5;
    
    private double distance;
    private double angle;

    public Auton(Realsense realsense, Driving drive) {

        this.realsense = realsense;
        this.drive = drive;
    }

    public void setState(State state) {
        this.state = state;
    }

    /*
    Positive and less than PI -> turn counterclockwise
    Positive and greater than PI -> turn clockwise
    Negative and abs value less than PI -> turn clockwise
    Negative and abs value more than PI -> turn counterclockwise
    */
    public void runToPoint() {		

		if(state == State.BEGIN) {
            // Moves into the next state. Also calculates the first turn angle 
            // and linear distance to the destination point
            state = State.FIRST_TURN;
            angle = realsense.getAbsoluteAngleToDestination();
            distance = realsense.getDistanceToDestination(); // Change this later to correct for drift
        }
        else if(state == State.FIRST_TURN) {
           
            double currentAngle = realsense.getCurrentAngle();
            // If the current angle is within the acceptable tolerances, move
            // into the next state
            if(angle - TURN_TOLERANCE < currentAngle && currentAngle < angle + TURN_TOLERANCE)  {
                state = State.FORWARD;
            } 
            else {
                // Otherwise, turn in the correct direction
                if((angle - currentAngle > 0 && angle-currentAngle < Math.PI) || (angle - currentAngle < 0 && Math.abs(angle-currentAngle) > Math.PI))
                    drive.driveSpeed(-speed, speed); // Change to PID later
                else
                    drive.driveSpeed(speed, -speed);
             
                System.out.println("Destination angle: " + angle + " Current angle: " + realsense.getCurrentAngle());
            }
        }
        else if(state == State.FORWARD) {
           // Drive forward until distance traveled exceeds the set distance
            if(realsense.getDistanceTraveled() < distance) {
                drive.driveStraight();
            }
            else {
                // Switch to the 2nd turn and get final angle
                state = State.SECOND_TURN;
                angle = realsense.getAngleFinal();
            }
            System.out.println("Distance:" + distance + " traveled:" + realsense.getDistanceTraveled());
        }
        else if(state == State.SECOND_TURN) {
            
            double currentAngle = realsense.getCurrentAngle();
            if(angle - TURN_TOLERANCE < currentAngle && currentAngle < angle + TURN_TOLERANCE)  {
                state = State.END; //Finished
            }
            else {
                if((angle - currentAngle > 0 && angle-currentAngle < Math.PI) || (angle - currentAngle < 0 && Math.abs(angle-currentAngle) > Math.PI))
                    drive.driveSpeed(-speed, speed); //Change to PID later
                else 
                    drive.driveSpeed(speed, -speed);
             
                System.out.println("Destination angle: " + angle + " Current angle: " + realsense.getCurrentAngle());
            }
        } else {
            drive.driveSpeed(0, 0);
        }
	}
}