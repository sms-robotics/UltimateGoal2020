package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import static org.firstinspires.ftc.teamcode.UtilMovement.inchesToTicksForQuadStraightDrive;
import static org.firstinspires.ftc.teamcode.UtilMovement.normalizeSpeedsForMinMaxValues;

public class MovementBehaviors {
    static final double HEADING_THRESHOLD = 2;
    static final double DISTANCE_THRESHOLD = 50;
    static final double SETTLING_TIME = 0.1;

    static final int GYRO = 1;
    static final int ENCODERS = 2;

    private static final double TURN_TICKS_PER_DEGREE = 14.5;
    private static final double TICKS_PER_MILLIMETER = 3.0;
    private static final double TURN_POWER = 0.125;
    private static final double TURN_ERROR_THRESHOLD_IN_DEGREES = 1;
    private static final double TURN_TIMEOUT_IN_MS = 4000;
    private static final double MIN_MOTOR_POWER = 0.0;
    private static final double MAX_MOTOR_POWER = 1.0;
    private static final double MIN_DRIVE_POWER = 0.01;
    private static final double MAX_DRIVE_POWER = 0.95;
    private static final double MIN_TURN_POWER = MIN_MOTOR_POWER;
    private static final double MAX_TURN_POWER = 0.5;
    private static final double DEFAULT_MOTOR_DRIVE_POWER = 0.5;
    private static final double DEFAULT_DRIVE_ANGLE = 0;

    private final ActionConveyor conveyor;
    private final ActionShooter shooter;
    private final ActionTrigger trigger;
    private final ActionWobbleArm wobbleArm;
    private final SensorIMU sensorImu;

    private ElapsedTime settlingtime = new ElapsedTime();
    boolean settlingtimeInitiated = false;
    private NerdPIDCalculator xPIDCalculator;
    private NerdPIDCalculator yPIDCalculator;
    private NerdPIDCalculator zPIDCalculator;
    private NerdPIDCalculator turnPIDCalculator;

    private enum MOTOR {
        FR,
        FL,
        RR,
        RL,
    };

    LinearOpMode opMode;
    HardwareUltimate robot;
    Telemetry telemetry;
    ElapsedTime runtime = new ElapsedTime();

    public MovementBehaviors(LinearOpMode opMode,
                             HardwareUltimate robot,
                             ActionConveyor conveyor,
                             ActionShooter shooter,
                             ActionTrigger trigger,
                             ActionWobbleArm wobbleArm,
                             SensorIMU sensorImu) {
        this.opMode = opMode;
        this.robot = robot;
        this.telemetry = opMode.telemetry;
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.trigger = trigger;
        this.wobbleArm = wobbleArm;
        this.sensorImu = sensorImu;
    }

    /*
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (in degrees relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */

    public double getError(double targetAngle) {
        double robotError;
        double currentAngle = sensorImu.getAngle();

        robotError = targetAngle - currentAngle;

        return robotError;
    }

    private double motorPowerCompensator(MOTOR motor, double power) {
        if (motor == MOTOR.FR) {
            return 1.0 * power;
        }
        if (motor == MOTOR.FL) {
            return 1.0 * power;
        }
        if (motor == MOTOR.RR) {
            return 1.0 * power;
        }
        if (motor == MOTOR.RL) {
            return 1.0 * power;
        }

        return power;
    }

    public void driveDistance(double distanceMm)  {
        driveDistance(distanceMm, DEFAULT_DRIVE_ANGLE);
    }

    public void driveDistance(double distanceMm, double angle)  {
        driveDistance(distanceMm, angle, DEFAULT_MOTOR_DRIVE_POWER);
    }

    public void driveDistance(double distanceMm, double angle, double power)
    {
        double clippedPower = Range.clip(power, MIN_DRIVE_POWER, MAX_DRIVE_POWER);
        setStraightDrivingModes();

        double targetRadAngle = Math.toRadians(angle);
        double radAngle = targetRadAngle;

        double angleX = Math.sin(radAngle);
        double angleY = Math.cos(radAngle);

        int targetPositionForward = (int)(distanceMm * TICKS_PER_MILLIMETER * angleY);
        int targetPositionRight = -(int)(distanceMm * TICKS_PER_MILLIMETER * angleX);
        int frontRightTarget = robot.frontRightDrive.getCurrentPosition() + targetPositionForward + targetPositionRight;
        int frontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + targetPositionForward - targetPositionRight;
        int rearRightTarget = robot.rearRightDrive.getCurrentPosition() + targetPositionForward - targetPositionRight;
        int rearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + targetPositionForward + targetPositionRight;

        robot.frontRightDrive.setTargetPosition(frontRightTarget);
        robot.frontLeftDrive.setTargetPosition(frontLeftTarget);
        robot.rearRightDrive.setTargetPosition(rearRightTarget);
        robot.rearLeftDrive.setTargetPosition(rearLeftTarget);

        double frontRightDriveAmount = (angleY - angleX);
        double frontLeftDriveAmount = (angleY + angleX);
        double rearRightDriveAmount = (angleY + angleX);
        double rearLeftDriveAmount = (angleY - angleX);

        double frontRightPower = motorPowerCompensator(MOTOR.FR,
            clippedPower * frontRightDriveAmount);
        double frontLeftPower = motorPowerCompensator(MOTOR.FL,
            clippedPower * frontLeftDriveAmount);
        double rearRightPower = motorPowerCompensator(MOTOR.RR,
            clippedPower * rearRightDriveAmount);
        double rearLeftPower = motorPowerCompensator(MOTOR.RL,
            clippedPower * rearLeftDriveAmount);

        // If an extremely small power is set, the motor might indicate "busy" for a very long time, so don't do that.
        if (Math.abs(frontRightPower) > MIN_DRIVE_POWER) robot.frontRightDrive.setPower(frontRightPower);
        if (Math.abs(frontLeftPower) > MIN_DRIVE_POWER) robot.frontLeftDrive.setPower(frontLeftPower);
        if (Math.abs(rearRightPower) > MIN_DRIVE_POWER) robot.rearRightDrive.setPower(rearRightPower);
        if (Math.abs(rearLeftPower) > MIN_DRIVE_POWER) robot.rearLeftDrive.setPower(rearLeftPower);

        while (opMode.opModeIsActive() && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
            // Update telemetry & Allow time for other processes to run
            // telemetry.addData("Drive for distance: ", distanceMm);
            // telemetry.addData("target: ", Math.toDegrees(targetRadAngle));
            // telemetry.addData("angleX: ", Math.toDegrees(angleX));
            // telemetry.addData("angleY: ", Math.toDegrees(angleY));
            // telemetry.update();
            opMode.idle();
        }
    }

    public void driveInDirection(double headingInRadians, double angleToFaceInRadians, double power, double turnPower)
    {
        double clippedPower = Range.clip(power, MIN_DRIVE_POWER, MAX_DRIVE_POWER);

        double leftX = -1 * Math.sin(headingInRadians);
        double leftY = Math.cos(headingInRadians);
        double rightX = angleToFaceInRadians * 50;

        double driveFactor = (power) / (power + turnPower);
        double turnFactor = 1 - driveFactor;
        double targetPower = Math.max(power, turnPower);

        // holonomic formulas
        double frontRight = ((leftY + leftX) * driveFactor) - (rightX * turnFactor);
        double frontLeft = ((leftY - leftX) * driveFactor) + (rightX * turnFactor);
        double backRight = ((leftY - leftX) * driveFactor) - (rightX * turnFactor);
        double backLeft = ((leftY + leftX) * driveFactor) + (rightX * turnFactor);

        double[] normalizedSpeeds = normalizeSpeedsForMinMaxValues(
            frontLeft,
            frontRight,
            backLeft,
            backRight,
            -1.0, 1.0,
            targetPower);

        // frontRight -= rightX;
        // frontLeft += rightX;
        // backRight -= rightX;
        // backLeft += rightX;

        // normalizedSpeeds = normalizeSpeedsForMinMaxValues(
        //     frontLeft,
        //     frontRight,
        //     backLeft,
        //     backRight,
        //     -1.0, 1.0,
        //     targetPower);

        // write the values to the motors
        robot.frontLeftDrive.setPower(normalizedSpeeds[0]);
        robot.frontRightDrive.setPower(normalizedSpeeds[1]);
        robot.rearLeftDrive.setPower(normalizedSpeeds[2]);
        robot.rearRightDrive.setPower(normalizedSpeeds[3]);

//        // clip the right/left values so that the values never exceed +/- 1
//        frontRight = Range.clip(frontRight, -1, 1) * clippedPower;
//        frontLeft = Range.clip(frontLeft, -1, 1) * clippedPower;
//        backLeft = Range.clip(backLeft, -1, 1) * clippedPower;
//        backRight = Range.clip(backRight, -1, 1) * clippedPower;
//
//        // write the values to the motors
//        robot.frontRightDrive.setPower(frontRight);
//        robot.frontLeftDrive.setPower(frontLeft);
//        robot.rearLeftDrive.setPower(backLeft);
//        robot.rearRightDrive.setPower(backRight);
    }

    public void motorsResetAndRunUsingEncoders(){
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Function to drive robot based on PIDs in X, Y and Z directions.
     *     *
     * @param   xDistance           - Distance to move in X direction.
     * @param   yDistance           - Distance to move in Y direction.
     * @param   zAngleToMaintain    - Angle to maintain.
     *
     */
    public void nerdPidDrive(double xDistance, double yDistance, double zAngleToMaintain, double targetPower, double timeOut) {
        final String funcName = "nerdPidDrive";

        //Speeds for assigning to 4 motors.

        double frontLeftSpeed;
        double frontRightSpeed;
        double rearLeftSpeed;
        double rearRightSpeed;

        //To hold the motor encoder ticks in each direction.

        int xTicks,yTicks;

        // Hold the PID values for X,Y and Z.
        double zpid, xpid, ypid;

        // Holds final motor powers to be sent to motors.
        double [] motorPowers;

        // Reset the timer
        runtime.reset();

        // Reset the Calculators
        xPIDCalculator.reset();
        yPIDCalculator.reset();
        zPIDCalculator.reset();

        sensorImu.startAccelerationIntegration();

        // Reset the motors so that the encoders are set to 0.
        motorsResetAndRunUsingEncoders();

        // Convert X and Y distances to corresponding encoder ticks.
        xTicks = xDistance != 0.0 ? (int)inchesToTicksForQuadStraightDrive(xDistance): 0;
        yTicks = yDistance != 0.0 ? (int)inchesToTicksForQuadStraightDrive(yDistance): 0;

        // Set PID targets for X, Y and Z
        // RobotLog.i("xTicks = %d, yTicks = %d , Angle %f", xTicks, yTicks, zAngleToMaintain);
        xPIDCalculator.setTarget(xTicks, findXDisplacement());
        yPIDCalculator.setTarget(yTicks, findYDisplacement());
        zPIDCalculator.setTarget(zAngleToMaintain, getZAngleValue());

        // Perform PID Loop until we reach the targets
        while (this.opMode.opModeIsActive() && !distanceTargetReached(xTicks, yTicks) && runtime.seconds() <= timeOut) {
            //Feed the input device readings to corresponding PID calculators:
            xpid = -1 * xPIDCalculator.getOutput(findXDisplacement(), ENCODERS);
            ypid = yPIDCalculator.getOutput(findYDisplacement(), ENCODERS);
            zpid = zPIDCalculator.getOutput(getZAngleValue(), GYRO);

            // Calculate Speeds based on Inverse Kinematics
            //RobotLog.i("xpid = %f, ypid = %f , zpid %f", xpid, ypid, zpid);

            frontLeftSpeed = ypid - xpid + zpid;
            frontRightSpeed = ypid + xpid - zpid;
            rearLeftSpeed = ypid + xpid + zpid;
            rearRightSpeed = ypid - xpid - zpid;

            // float frontRight = leftY + leftX - gamepad1RightX;
            // float frontLeft = leftY - leftX + gamepad1RightX;
            // float backRight = leftY - leftX - gamepad1RightX;
            // float backLeft = leftY + leftX + gamepad1RightX;

            // Normalize the Motor Speeds for Min and Max Values
            motorPowers = normalizeSpeedsForMinMaxValues(
                frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed, 0.0, 1, targetPower);

//            RobotLog.i("Speeds (FL:%f, FR:%f, RL:%f, RR:%f)",
//                motorPowers[0],
//                motorPowers[1],
//                motorPowers[2],
//                motorPowers[3]);

            // Set Powers to corresponding Motors
            robot.frontLeftDrive.setPower(motorPowers[0]);
            robot.frontRightDrive.setPower(motorPowers[1]);
            robot.rearLeftDrive.setPower(motorPowers[2]);
            robot.rearRightDrive.setPower(motorPowers[3]);
        }

        sensorImu.stopAccelerationIntegration();

        //Brake once the PID loop is complete
        RobotLog.d("NerdBOT  - Displacement Before Stop : %s|xTarget | yTarget | xDisplacement | yDisplacement ", funcName);
        RobotLog.d("NerdBOT  - Displacement Before Stop : %s|%d|%d|%f|%f ", funcName, xTicks, yTicks, findXDisplacement(), findYDisplacement());
        stopWheels();
    }

    public void driveForTime(double forward, double right, double msToRun)
    {
        robot.frontRightDrive.setPower(forward + right);
        robot.frontLeftDrive.setPower(forward - right);
        robot.rearRightDrive.setPower(forward - right);
        robot.rearLeftDrive.setPower(forward + right);
        double intRunTime = runtime.milliseconds() + msToRun;

        while (runtime.milliseconds() < intRunTime) {
            telemetry.addData("drive-l", forward);
            telemetry.addData("drive-r", right);
            telemetry.addData("time", runtime.milliseconds());
            telemetry.update();
            opMode.idle();
        }

        robot.frontRightDrive.setPower(0.0);
        robot.frontLeftDrive.setPower(0.0);
        robot.rearRightDrive.setPower(0.0);
        robot.rearLeftDrive.setPower(0.0);
    }

    public void motorsSetMode(DcMotor.RunMode runMode){
        robot.frontLeftDrive.setMode(runMode);
        robot.frontRightDrive.setMode(runMode);
        robot.rearLeftDrive.setMode(runMode);
        robot.rearRightDrive.setMode(runMode);
    }

    public void nerdPidTurn(double targetAngle) {
        double pidvalue;
        double [] motorPowers;

        double frontLeftSpeed=0, frontRightSpeed=0, rearRightSpeed=0, rearLeftSpeed=0;

        runtime.reset();

        turnPIDCalculator.setTarget(targetAngle, getZAngleValue());

        motorsSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!opMode.isStopRequested() && runtime.milliseconds() <= TURN_TIMEOUT_IN_MS) {
            double angle = getZAngleValue();

            pidvalue = turnPIDCalculator.getOutput(angle, GYRO);

            frontLeftSpeed =  -pidvalue;
            frontRightSpeed = pidvalue;
            rearLeftSpeed = -pidvalue;
            rearRightSpeed = pidvalue;

            double targetTurnPower = 0.1;//Math.abs(angle - targetAngle) / 90.0;

            motorPowers = normalizeSpeedsForMinMaxValues(
                frontLeftSpeed,
                frontRightSpeed,
                rearLeftSpeed,
                rearRightSpeed,
                MIN_TURN_POWER, MAX_TURN_POWER, // XXX
                targetTurnPower);

            robot.frontLeftDrive.setPower(motorPowers[0]);
            robot.frontRightDrive.setPower(motorPowers[1]);
            robot.rearLeftDrive.setPower(motorPowers[2]);
            robot.rearRightDrive.setPower(motorPowers[3]);
        }

        stopWheels();
    }

    public void startTurningLeft(double power) {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rearLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.frontRightDrive.setPower(power);
        robot.frontLeftDrive.setPower(power);
        robot.rearRightDrive.setPower(power);
        robot.rearLeftDrive.setPower(power);
    }

    public void startTurningRight(double power) {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rearRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.frontRightDrive.setPower(power);
        robot.frontLeftDrive.setPower(power);
        robot.rearRightDrive.setPower(power);
        robot.rearLeftDrive.setPower(power);
    }

    public void stopWheels() {
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
    }

    public void turnTo(double degrees) {
        ElapsedTime elapsedTime = new ElapsedTime();

        while (opMode.opModeIsActive() && elapsedTime.milliseconds() < TURN_TIMEOUT_IN_MS) {
            double error = getError(degrees);

            double targetTurnPower = Math.abs(error) / 90.0;
            double normalizedTurnPower = Range.clip(targetTurnPower, MIN_TURN_POWER, MAX_TURN_POWER);

            // telemetry.addData("Error is: ", error);
            // telemetry.update();
            if (Math.abs(error) < TURN_ERROR_THRESHOLD_IN_DEGREES) {
                stopWheels();
                Thread.yield();
                // opMode.idle();
                break;
            } else if (error < 0) {
                startTurningLeft(normalizedTurnPower);
            } else {
                startTurningRight(normalizedTurnPower);
            }
            Thread.yield();
            // opMode.idle();
        }

        stopWheels();
    }

    public void setRunUsingEncoderMode() {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRunToPositionMode() {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setStraightDrivingModes() {
        setRunToPositionMode();
        robot.frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Function to find out the Robot travel distance in X direction.
    double findXDisplacement() {
        // Position position = sensorImu.getPosition();
        // double xDisplacement = position.x;
        // double yDisplacement = position.y;
        // double zDisplacement = position.z;
        double fl = robot.frontLeftDrive.getCurrentPosition();
        double fr = robot.frontRightDrive.getCurrentPosition();
        double rl = robot.rearLeftDrive.getCurrentPosition();
        double rr = robot.rearRightDrive.getCurrentPosition();

        double xDisplacement = (fl - fr - rl + rr)/4.0;

//        RobotLog.i("xDisplacement %f (%f, %f, %f, %f)", xDisplacement, fl, fr, rl, rr);
        // RobotLog.i("xDisplacement %f, (%f), %f", xDisplacement, yDisplacement, zDisplacement);
        return xDisplacement;
    }

    //Function to find out the Robot travel distance in Y direction.
    double findYDisplacement() {
        double fl = robot.frontLeftDrive.getCurrentPosition();
        double fr = robot.frontRightDrive.getCurrentPosition();
        double rl = robot.rearLeftDrive.getCurrentPosition();
        double rr = robot.rearRightDrive.getCurrentPosition();

        double yDisplacement = (fl + fr + rl + rr)/4.0;

//        RobotLog.i("yDisplacement %f (%f, %f, %f, %f)", yDisplacement, fl, fr, rl, rr);

        // Position position = sensorImu.getPosition();

        // return position.x;

        // return (robot.frontLeftDrive.getCurrentPosition() + robot.frontRightDrive.getCurrentPosition()
        //     + robot.rearLeftDrive.getCurrentPosition() + robot.rearRightDrive.getCurrentPosition())/4.0;
        return yDisplacement;
    }


    public void initializeZPIDCalculator(double kP, double kI, double kD, boolean debugFlag){

        this.zPIDCalculator = new NerdPIDCalculator("zPIDCalculator", kP, kI, kD);
        this.zPIDCalculator.setDebug(debugFlag);
    }

    public void initializeXPIDCalculator(double kP, double kI, double kD,boolean debugFlag){

        this.xPIDCalculator = new NerdPIDCalculator("xPIDCalculator", kP, kI, kD);
        this.xPIDCalculator.setDebug(debugFlag);
    }
    public void initializeYPIDCalculator(double kP, double kI, double kD,boolean debugFlag){

        this.yPIDCalculator = new NerdPIDCalculator("yPIDCalculator", kP, kI, kD);
        this.yPIDCalculator.setDebug(debugFlag);
    }

    public void initializeTurnPIDCalculator(double kP, double kI, double kD,boolean debugFlag){

        this.turnPIDCalculator = new NerdPIDCalculator("turnPIDCalculator", kP, kI, kD);
        this.turnPIDCalculator.setDebug(debugFlag);

    }

    //Function to find if the robot reached the desired target distance.
    //If desired distance is reached, it also checks if it is withing z angle tolerance.

    boolean distanceTargetReached( int xTicks, int yTicks){

        boolean onDistanceTarget = false;
        boolean onFinalTarget = false;
        if(xTicks == 0 && yTicks != 0){
            if(Math.abs(yTicks) - Math.abs(findYDisplacement()) <= DISTANCE_THRESHOLD){

                onDistanceTarget = true;

            }
        }else if(yTicks == 0 && xTicks != 0 ){
            if(Math.abs(xTicks) - Math.abs(findXDisplacement()) <= DISTANCE_THRESHOLD){

                onDistanceTarget = true;
            }

        }else{
            if((Math.abs(yTicks) - Math.abs(findYDisplacement()) <= DISTANCE_THRESHOLD) && (Math.abs(xTicks) - Math.abs(findXDisplacement()) <= DISTANCE_THRESHOLD)){

                onDistanceTarget = true;
            }

        }
        if(onFinalTarget == false) {
            onFinalTarget = onDistanceTarget;
        }

        if((onFinalTarget) && !settlingtimeInitiated)  {
            settlingtime.reset();
            settlingtimeInitiated = true;
        }

//        if (onFinalTarget && !timerStarted){
//            settlingtime.reset();
//            timerStarted = true;
//        }
//
//        if (onFinalTarget && !(settlingtime.seconds() < 0.2))
//              return  onFinalTarget;
//        else
//            return false;

        if(onFinalTarget){
            if(settlingtime.seconds() < SETTLING_TIME){
                return false;
            }
            else{
                return true;
            }
        }
        else {
            return onFinalTarget;
        }
    }

    public void waitForTimeInMilliseconds(double millisecondsToWait) {
        double msRunTime = runtime.milliseconds() + millisecondsToWait;

        while (runtime.milliseconds() < msRunTime && opMode.opModeIsActive()) {
            // opMode.idle();
            Thread.yield();
        }
    }

    public double getZAngleValue() {
        return sensorImu.getAngle();
    }

    public void resetAngle() {
        sensorImu.resetAngle();
    }

    public void turnOnConveyor(double parameter) {
        conveyor.turnOnAtPower(parameter);
    }

    public void turnOffCoveyor() {
        conveyor.turnOff();
    }

    public void turnOnShooter(double parameter) {
        shooter.turnOnAtPower(parameter);
    }

    public void turnOffShooter() {
        shooter.turnOff();
    }

    public void lowerWobbleArmCompletely() {
        wobbleArm.lowerArmFully();
    }

    public void raiseWobbleArmCompletely() {
        wobbleArm.raiseArmFully();
    }

    public void moveWobbleArmToPosition(double positionAsPercentage, double atPower) {
        wobbleArm.moveArmToPosition(positionAsPercentage, atPower);
    }

    public void fireTrigger(double millisecondsToWaitForTriggerSweep) {
        trigger.fire();

        waitForTimeInMilliseconds(millisecondsToWaitForTriggerSweep);

        trigger.resetPosition();

        waitForTimeInMilliseconds(millisecondsToWaitForTriggerSweep);
    }

    public void waitForWobbleArm(double timeoutInMs) {
        wobbleArm.waitForWobbleArm((int)timeoutInMs);
    }
}
