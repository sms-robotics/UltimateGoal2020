package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class UtilMovement {
    private static final double GEAR_RATIO = 40.0/1.0;  // Gear ratio
    private static final double ticksPerRotation = 560.0;
    private static final double wheelDiameterInMm = 90;
    private static final double wheelMountAngleInDeg = 45.0;

    static public double[] normalizeSpeedsForMinMaxValues(double frontLeftSpeed, double frontRightSpeed, double rearLeftSpeed, double rearRightSpeed, double minSpeed, double maxSpeed, double targetPower) {

        double max = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed)), Math.max(Math.abs(rearRightSpeed), Math.abs(rearLeftSpeed)));
        if (max > maxSpeed) {
            frontLeftSpeed = (frontLeftSpeed * maxSpeed)/max;
            frontRightSpeed = (frontRightSpeed * maxSpeed)/ max;
            rearRightSpeed = (rearRightSpeed * maxSpeed)/max;
            rearLeftSpeed = (rearLeftSpeed * maxSpeed)/max;
        }

        //If there is min speed other than 0.
        // Preserve the sign of each speed for each motor
        int leftSign, rightSign, leftSignB, rightSignB;

        leftSign = (int)Math.signum(frontLeftSpeed);
        rightSign = (int)Math.signum(frontRightSpeed);
        leftSignB =(int) Math.signum(rearRightSpeed);
        rightSignB = (int)Math.signum(rearLeftSpeed);

        // Assign min speed with correct sign

        if (Math.abs(frontLeftSpeed) < minSpeed) frontLeftSpeed = minSpeed*leftSign;
        if (Math.abs(frontRightSpeed) < minSpeed) frontRightSpeed = minSpeed*rightSign;
        if (Math.abs(rearRightSpeed) < minSpeed) rearRightSpeed = minSpeed*leftSignB;
        if (Math.abs(rearLeftSpeed) < minSpeed) rearLeftSpeed = minSpeed*rightSignB;

        double[] normalizedSpeeds = {
            Range.clip(frontLeftSpeed * targetPower, -1, 1),
            Range.clip(frontRightSpeed * targetPower, -1, 1),
            Range.clip(rearLeftSpeed * targetPower, -1, 1),
            Range.clip(rearRightSpeed * targetPower, -1, 1),
        };

        return  normalizedSpeeds;
    }

    static public int inchesToTicksForQuadStraightDrive(double straightDistance){
        int ticks;

        // circumference of the wheel

        double circum = wheelDiameterInMm * Math.PI;

        //Since the wheels are mounted at an angle, do the math for wheel distance to travel
        double wheelDistanceToTravel = (straightDistance * Math.cos(Math.toRadians(wheelMountAngleInDeg))) * GEAR_RATIO;

        //Find the number of rotations for wheel distance to travel.
        double numberOfWheelRotations = wheelDistanceToTravel/circum;

        //Convert number of rotation into motor encoder ticks
        ticks = (int)(Math.round(ticksPerRotation * numberOfWheelRotations));

        return ticks;
    }
}
