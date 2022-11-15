package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SwerveModule {
    public WPI_TalonFX speedMotor;
    public WPI_TalonSRX angleMotor;
    public final int zeroPoint;
    public SwerveModule (int speedMotordeviceid, int angleMotordeviceid, final int zeroAngle, boolean inverted) {
        speedMotor = new WPI_TalonFX(speedMotordeviceid);
        angleMotor = new WPI_TalonSRX(angleMotordeviceid);
        zeroPoint = zeroAngle;
        angleMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        angleMotor.configFeedbackNotContinuous(false, 0);
        setPID(1, 0, 0);
        speedMotor.setInverted(inverted);
    }
    public void setSpeed(double speed) {
        speedMotor.set(speed);
    }
    public void setAngle(double angle) {
        int angleI = (int)(angle * 2048);
        angleI += zeroPoint;
        angleMotor.set(ControlMode.Position, angleI);
    }

    /**
     * Set the angle and speed of the swerve module.
     * 
     * @param angle Angle of the wheel from negative pi to pi
     * @param speed Velocity in m/s
     */
    public void setAngleAndSpeed(double angle, double speed) {
        int angleI = (int)(angle / Math.PI * 2048 + 2048 + zeroPoint);
        int current_angle = (int)(angleMotor.getSelectedSensorPosition());
        int delta_angle = (angleI - current_angle) % 4096;
        int target_angle;
        if (delta_angle <= 1024) {
            target_angle = current_angle + delta_angle;
        }
        else if (delta_angle <= 3072) {
            speed *= -1;
            target_angle = current_angle + delta_angle - 2048;
        }
        else {
            target_angle = current_angle + delta_angle + 4096;
        }

        if (speed != 0) {
            angleMotor.set(ControlMode.Position, target_angle);
        }

        speedMotor.set(speed);

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
    public double getAngle() {
        return angleMotor.getSelectedSensorPosition();
    }
    public double getAnglePowerOutput() {
        return angleMotor.getStatorCurrent();
    }
    public double getDriveTemp() { 
        return speedMotor.getTemperature();
    }
}
