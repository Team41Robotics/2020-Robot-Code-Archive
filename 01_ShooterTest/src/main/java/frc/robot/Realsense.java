package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;

class Realsense {
	private NetworkTable realsense;
	private NetworkTableEntry sinStartCam, cosStartCam, thetaFieldRobot, xFieldRobot, yFieldRobot;

	private boolean firstRun;

	private double xStart, yStart, xFinal, yFinal, angleFinal;

	public Realsense() {
		realsense = NetworkTableInstance.getDefault().getTable("realsense");

		xFieldRobot = realsense.getEntry("xFieldRobot"); //X coordinate of robot in relation to field
		yFieldRobot = realsense.getEntry("yFieldRobot"); //Y coordinate of robot in relation to field
		thetaFieldRobot = realsense.getEntry("thetaFieldRobot"); //Angle of robot in relation to field 
		sinStartCam = realsense.getEntry("sinStartCam"); //Cos of robot in relation to the field
		cosStartCam = realsense.getEntry("cosStartCam"); //Cos of robot in relation to the field

		firstRun = true;
	}

	public void runRealsense(Joystick controller) {
		if(firstRun)
			realsenseInit();
	}

	public double getAngleFinal() {
		return angleFinal;
	}

	public void realsenseInit(){
		xStart = xFieldRobot.getDouble(0.0);
		yStart = yFieldRobot.getDouble(0.0);

		firstRun = false;
	}

	public double getAbsoluteAngleToDestination(){
		double Dx = xFinal - xStart;
		double Dy = yFinal - yStart;

		double relAngle = Math.atan(Dy/Dx);
		
		// Account for angle being in quadrant II or III 
		double absAngle = Dx < 0 ? Math.PI + relAngle : relAngle;

		// Transforms angle from [0, 2PI] into [-PI, PI]
		if(absAngle > Math.PI) {
			absAngle = absAngle - (2*Math.PI);
		}

		return absAngle; // In radians
	}

	public double getDistanceToDestination() {
		double Dx = xFinal - xStart;
		double Dy = yFinal - yStart;

		// Just plug into the distance formula
		double distance = Math.sqrt(Math.pow(Dx,2) + Math.pow(Dy,2)); 

		return distance;
	}

	public double getDistanceTraveled() {
		double Dx = xFieldRobot.getDouble(0.0) - xStart;
		double Dy = yFieldRobot.getDouble(0.0) - yStart;

		// Just plug into the distance formula
		double distance = Math.sqrt(Math.pow(Dx,2) + Math.pow(Dy,2)); 

		return distance;
	}

	public double getCurrentAngle() {
		// double sinSC = sinStartCam.getDouble(0.0);
		// double cosSC = cosStartCam.getDouble(0.0);
		// double sinTar = Math.sin(angleFinal);
		// double cosTar = Math.cos(angleFinal);
		// double[][] rotationStartCam = {{cosSC, sinSC},{-sinSC, cosSC}};
		// double[][] rotationTarget = {{cosTar, -sinTar},{sinTar, cosTar}};

		// return Math.acos(multiplyMatrix(rotationTarget, rotationStartCam)[0][0]);
		return thetaFieldRobot.getDouble(0.0);
	}

	public void setDestination(Pose dest){
		xFinal = dest.x;
		yFinal = dest.y;
		angleFinal = dest.theta;

		xStart = xFieldRobot.getDouble(0.0);
		yStart = yFieldRobot.getDouble(0.0);
	} 

	public static double[][] multiplyMatrix(double[][] a, double[][] b) {
		double[][] product = new double[2][2];
		for(int i = 0; i < 2; i++) {
			for (int j = 0; j < 2; j++) {
				for (int k = 0; k < 2; k++) {
					product[i][j] += a[i][k] * b[k][j];
				}
			}
		}
		return product;
	}

}