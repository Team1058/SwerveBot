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
        frontLeft = new SwerveModule(0, 1, frontLeftStartPoint);
        frontRight = new SwerveModule(2, 3, frontRightStartPoint);
        backLeft = new SwerveModule(4, 5, backLeftStartPoint);
        backRight = new SwerveModule(6, 7, backRightStartPoint);
    }
    public void drive (double x1, double y1, double x2) {

        double r = Math.sqrt ((L * L) + (W * W));
        y1 *= -1;

        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r); 
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

        double backRightAngle = Math.atan2 (a, d) / Math.PI;
        double backLeftAngle = Math.atan2 (a, c) / Math.PI;
        double frontRightAngle = Math.atan2 (b, d) / Math.PI;
        double frontLeftAngle = Math.atan2 (b, c) / Math.PI;
 
        frontLeft.setSpeed(frontLeftSpeed);
        frontRight.setSpeed(frontRightSpeed);
        backLeft.setSpeed(backLeftSpeed);
        backRight.setSpeed(backRightSpeed);

        if (x1 != 0 || x2 != 0 || y1 != 0) {
            frontLeft.setAngle(frontLeftAngle);
            frontRight.setAngle(frontRightAngle);
            backLeft.setAngle(backLeftAngle);
            backRight.setAngle(backRightAngle);
            SmartDashboard.putNumber("backLeftAngleSet", backLeftAngle);
            SmartDashboard.putNumber("backRightAngleSet", backRightAngle);
            SmartDashboard.putNumber("frontLeftAngleSet", frontLeftAngle);
            SmartDashboard.putNumber("frontRightAngleSet", frontRightAngle);
        }
        
        SmartDashboard.putNumber("x1", x1);
        SmartDashboard.putNumber("x2", x2);
        SmartDashboard.putNumber("y1", y1);
        
    }
    public void printAngles() {
        SmartDashboard.putNumber("frontLeftAngle", frontLeft.getAngle());
        SmartDashboard.putNumber("frontRightAngle", frontRight.getAngle());
        SmartDashboard.putNumber("backLeftAngle", backLeft.getAngle());
        SmartDashboard.putNumber("backRightAngle", backRight.getAngle());
    }

}
