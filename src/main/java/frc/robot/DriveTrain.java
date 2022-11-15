package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;
    final int frontRightStartPoint = 2382;
    final int frontLeftStartPoint = 2706;
    final int backRightStartPoint = 2995;
    final int backLeftStartPoint = 296;
    public final double L = 21.5;
    public final double W = 21.5;
    public DriveTrain(){
        frontLeft = new SwerveModule(0, 1, frontLeftStartPoint, true);
        frontRight = new SwerveModule(2, 3, frontRightStartPoint, false);
        backLeft = new SwerveModule(4, 5, backLeftStartPoint, true);
        backRight = new SwerveModule(6, 7, backRightStartPoint, false);
    }
    public void drive (double x1, double y1, double x2) {
        if (Math.abs(x1)<.1){
            x1 = 0;
            
        }
        if (Math.abs(y1)<.1){
            y1 = 0;
            
        }
        if (Math.abs(x2)<.1){
            x2 = 0;
            
        }

        double r = Math.sqrt ((L * L) + (W * W));

        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r); 
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

        double backRightAngle = Math.atan2 (a, d);
        double backLeftAngle = Math.atan2 (a, c);
        double frontRightAngle = Math.atan2 (b, d);
        double frontLeftAngle = Math.atan2 (b, c);
 
        frontRight.setAngleAndSpeed(frontRightAngle, frontRightSpeed);
        frontLeft.setAngleAndSpeed(frontLeftAngle, frontLeftSpeed);
        backRight.setAngleAndSpeed(backRightAngle, backRightSpeed);
        backLeft.setAngleAndSpeed(backLeftAngle, backLeftSpeed);
        
        SmartDashboard.putNumber("x1", x1);
        SmartDashboard.putNumber("x2", x2);
        SmartDashboard.putNumber("y1", y1);
        
    }

    public void printPowerConsumption(){
        SmartDashboard.putNumber("frontLeftAnglePower", frontLeft.getAnglePowerOutput());
        SmartDashboard.putNumber("frontRightAnglePower", frontRight.getAnglePowerOutput());
        SmartDashboard.putNumber("backLeftAnglePower", backLeft.getAnglePowerOutput());
        SmartDashboard.putNumber("backRightAnglePower", backRight.getAnglePowerOutput());


        }

    public void printAngles() {
        SmartDashboard.putNumber("frontLeftAngle", frontLeft.getAngle());
        SmartDashboard.putNumber("frontRightAngle", frontRight.getAngle());
        SmartDashboard.putNumber("backLeftAngle", backLeft.getAngle());
        SmartDashboard.putNumber("backRightAngle", backRight.getAngle());


        SmartDashboard.putNumber("normalizeFrontLeftAngle", frontLeft.getAngle()%360);
        SmartDashboard.putNumber("normalizeFrontRightAngle",frontRight.getAngle()%360);
        SmartDashboard.putNumber("normalizeBackLeftAngle", backLeft.getAngle()%360);
        SmartDashboard.putNumber("normalizeBackRightAngle", backRight.getAngle()%360);
    }
    public void printTemp() {
        SmartDashboard.putNumber("frontLeftMotorTemp", frontLeft.getDriveTemp());
        SmartDashboard.putNumber("frontRightMotorTemp", frontRight.getDriveTemp());
        SmartDashboard.putNumber("backLeftMotorTemp", backLeft.getDriveTemp());
        SmartDashboard.putNumber("backRightMotorTemp", backRight.getDriveTemp());


    }

}
