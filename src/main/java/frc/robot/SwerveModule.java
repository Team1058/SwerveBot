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
    public SwerveModule (int speedMotordeviceid, int angleMotordeviceid, final int zeroAngle) {
        speedMotor = new WPI_TalonFX(speedMotordeviceid);
        angleMotor = new WPI_TalonSRX(angleMotordeviceid);
        zeroPoint = zeroAngle;
        angleMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        angleMotor.configFeedbackNotContinuous(false, 0);
        setPID(1, 0, 0);
    }
    public void setSpeed(double speed) {
        speedMotor.set(speed);
    }
    public void setAngle(double angle) {
        int angleI = (int)(angle * 2048);
        angleI += zeroPoint;
        angleMotor.set(ControlMode.Position, angleI);
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
