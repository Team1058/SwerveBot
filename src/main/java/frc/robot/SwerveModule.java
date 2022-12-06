package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    public WPI_TalonFX speedMotor;
    public WPI_TalonSRX angleMotor;
    public String name;
    public final int angleTransform;
    public SwerveModule (int speedMotordeviceid, int angleMotordeviceid, final int zeroAngle, boolean inverted, String _name) {
        speedMotor = new WPI_TalonFX(speedMotordeviceid);
        angleMotor = new WPI_TalonSRX(angleMotordeviceid);
        angleTransform = 2048 - zeroAngle;
        angleMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        angleMotor.configFeedbackNotContinuous(false, 0);
        setPID(1, 0, 0);
        speedMotor.setInverted(inverted);
        name = _name;

    }
    /**
     * Transform an angle from controller coordinates to motor coordinates
     * 
     * @param controller_angle angle given from 0 to 4095
     * @return transformed angle from 0 to 4095
     */
    private int transformAngle(int controller_angle) {
        return Math.floorMod(controller_angle - angleTransform, 4096);
    }
    /**
     * Converts an angle in radians to encoder ticks
     * 
     * @param angle_rad angle from -PI to PI
     * @return angle from 0 to 4095
     */
    private static int angleRadToTicks(double angle_rad) {
        return Math.floorMod(Math.round((angle_rad + Math.PI) / Math.PI  * 2048), 4096); // The floormod is used to turn 4096 into 0
    }
    /**
     * Set the angle and speed of the swerve module.
     * 
     * @param angle Angle of the wheel from negative pi to pi
     * @param speed Velocity in m/s
     */
    public void setAngleAndSpeed(double angle, double speed) {
        long inputTargetAngle_ticks = angleRadToTicks(angle); // use this to print out what angle is fed to this function in encoder ticks
        long sensorTargetAngle_ticks =  transformAngle(angleRadToTicks(angle)); // range of 0 to 4095
        int current_angle = (int)(angleMotor.getSelectedSensorPosition()); // current position in raw sensor units
        int delta_angle = Math.floorMod(sensorTargetAngle_ticks - current_angle, 4096); //difference between current angle and target angle
        int target_angle;

        if (delta_angle <= 1024) {
            target_angle = current_angle + delta_angle;
        }
        else if (delta_angle <= 3072) {
            speed *= -1;
            target_angle = current_angle + delta_angle - 2048;
        }
        else {
            target_angle = current_angle + delta_angle - 4096;
        }
        if (speed != 0) {
            angleMotor.set(ControlMode.Position, target_angle);
            SmartDashboard.putNumber(name + " - commanded value", target_angle);
        }
    
        speedMotor.set(speed);

        SmartDashboard.putNumber(name + " - target angle", inputTargetAngle_ticks);

    }
    public void printAngle()
    {
        System.out.println(angleMotor.getSelectedSensorPosition());
    
    }
    public void setPID(double P, double I, double D){
        int kPIDLoopIdx = 0;
		angleMotor.config_kP(kPIDLoopIdx, P, 0);
		angleMotor.config_kI(kPIDLoopIdx, I, 0);
		angleMotor.config_kD(kPIDLoopIdx, D, 0);
    }
    /**
     * 
     * @return The current encoder position normalised to be in range of 0 - 4095
     */
    public int getAngleTicks() {
        return Math.floorMod((int)(angleMotor.getSelectedSensorPosition()), 4096);
    }
    public double getAnglePowerOutput() {
        return angleMotor.getStatorCurrent();
    }
    public double getDriveTemp() { 
        return speedMotor.getTemperature();
    }
    public int reverseTransformAngle(int angle) {
        return Math.floorMod(angle + angleTransform, 4096);
    }

    public void printStats() {
        double encoderPosition = angleMotor.getSelectedSensorPosition();
        int normaliseAngle = reverseTransformAngle(getAngleTicks()) ;
        double motorTemperature = getDriveTemp();

        SmartDashboard.putNumber(name + " - actual commanded value", angleMotor.get());
        SmartDashboard.putNumber(name + " - encoder position", encoderPosition);
        SmartDashboard.putNumber(name + " - normalised angle", normaliseAngle);
        SmartDashboard.putNumber(name + " - motor temperature", motorTemperature);
    }
}
