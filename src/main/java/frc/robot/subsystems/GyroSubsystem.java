package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * This class provides an interface for the Pigeon Gyro
 */
public class GyroSubsystem extends SubsystemBase implements Gyro {

	private PigeonIMU pigeon = new PigeonIMU(0);
	double zeroAngle;
	/**
	 * this is the constructor of the class
	 */
	public GyroSubsystem() {
		reset();
	}
	
	@Override
	public void periodic() {
	  // This method will be called once per scheduler run
	  displayAngle();
	}
    
    public void displayAngle() {
        SmartDashboard.putNumber("Angle:", getAngle());
    }

	@Override
	public void calibrate() {
	}
	/**
	 * sets current angle to zero
	 */
	@Override
	public void reset() {
		zeroAngle = getRawAngle();
	}
	/**
	 * calculates current angle
	 * returns a double value of the current angle
	 */
	@Override
	public double getAngle() {
		return getRawAngle() - zeroAngle;
	}
	/**
	 * gets the rate of the robot
	 * returns double value of the robot rate
	 */
	@Override
	public double getRate() {
		return 0;
	}

	/**
	 * calculates total rotation of robot
	 * @return double total angle
	 */
	private double getRawAngle() {
		double [] ypr = new double [3];
		pigeon.getYawPitchRoll(ypr);
		return -ypr[0];
	}

	@Override
	public void close() throws Exception {

	}
	

}