package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class SensorIMU {
    private static final String TAG = "SensorIMU";
    private static final double MAX_TIME_TO_ALLOW_CALIBRATION_MS = 2000;
    private final BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle = 0;

    public SensorIMU(BNO055IMU imu) {
        this.imu = imu;
    }

    public void initialize(LinearOpMode opMode) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        ElapsedTime elapsedTime = new ElapsedTime();

        if (opMode != null) {
             // make sure the imu gyro is calibrated before continuing.
             while (!opMode.isStopRequested() && !imu.isGyroCalibrated() && elapsedTime.milliseconds() < MAX_TIME_TO_ALLOW_CALIBRATION_MS)
             {
                 opMode.sleep(50);
                 opMode.idle();
             }
        }
    }

    public void startAccelerationIntegration() {
        imu.startAccelerationIntegration(new Position(DistanceUnit.MM, 0, 0, 0, 0), new Velocity(DistanceUnit.MM, 0.0, 0.0, 0.0, 0), 100);
    }

    public void stopAccelerationIntegration() {
        imu.stopAccelerationIntegration();
    }

    public Position getPosition() {
        return imu.getPosition();
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = lastAngles;

        try {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        } catch (Exception e) {
            RobotLog.ee(TAG, e, "Couldn't get Orientation. Using Last.");
        }

        double deltaAngle = lastAngles.firstAngle - angles.firstAngle;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        try {
            this.lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        } catch (Exception e) {
            RobotLog.ee(TAG, e, "Couldn't get Orientation. Using Last.");
        }

        globalAngle = 0;
    }

}
