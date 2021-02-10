package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.UltimateAuton.InstructionType.*;

@Autonomous(name="Auton", group="Pushbot")
public class UltimateAuton extends LinearOpMode {

    private static final double TURN_TICKS_PER_DEGREE = 14.5;
    private static final double TICKS_PER_MILLIMETER = 8.0;
    private static final double TURN_POWER = 0.5;
    private static final double DRIVE_DISTANCE_POWER = 0.5;
    private static final double TURN_ERROR_THRESHOLD = 0.2;

    /* Declare OpMode members. */
    UltimateHardware robot = new UltimateHardware();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime();

    List<Instruction> instructions = Arrays.asList(
            new Instruction(DRIVE_DISTANCE, 100, 0),
            new Instruction(TURN_TO, -90),
            new Instruction(DRIVE_DISTANCE, 100, 0),
            new Instruction(TURN_TO, 180),
            new Instruction(DRIVE_DISTANCE, 100, 0),
            new Instruction(TURN_TO, 90),
            new Instruction(DRIVE_DISTANCE, 100, 0),
            new Instruction(TURN_TO, 0),
            new Instruction(DRIVE_DISTANCE, 100, 0),
            new Instruction(TURN_TO, -90),
            new Instruction(DRIVE_DISTANCE, 100, 0),
            new Instruction(TURN_TO, 180),
            new Instruction(DRIVE_DISTANCE, 100, 0),
            new Instruction(TURN_TO, 90),
            new Instruction(DRIVE_DISTANCE, 100, 0),
            new Instruction(TURN_TO, 0)
    );

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true);
        //move to hardware code and only run in AUTON

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        float powerReducer = 0.5f;
        // Wait for the game to start (driver presses PLAY)


        int instructionIndex = 0;

        // Assuming we're not going to mix and match modes for now.
        setRunToPositionMode();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double error = getError(0);
            telemetry.addData("direction from 0: ", error);

            if (instructionIndex >= instructions.size()) {
                telemetry.addData("No more instructions", "");
            } else {
                Instruction currentInstruction = instructions.get(instructionIndex);
                telemetry.addData("Current Instruction: ", currentInstruction.instructionType.name());
                currentInstruction.execute(this);
                instructionIndex++;
            }
            telemetry.update();
            idle();
        }
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
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public void driveDistance(double distanceMm, double angle)
    {
        telemetry.addData("Drive for distance: ", distanceMm);
        telemetry.update();

        setStraightDrivingModes();

        // TODO: Use real angle - Starting out with angle == 0
        int targetPositionForward = (int)(distanceMm * TICKS_PER_MILLIMETER);
        int targetPositionRight = 0;
        int frontRightTarget = robot.frontRightDrive.getCurrentPosition() + targetPositionForward + targetPositionRight;
        int frontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + targetPositionForward - targetPositionRight ;
        int rearRightTarget = robot.rearRightDrive.getCurrentPosition() + targetPositionForward - targetPositionRight;
        int rearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + targetPositionForward - targetPositionRight;

        robot.frontRightDrive.setTargetPosition(frontRightTarget);
        robot.frontLeftDrive.setTargetPosition(frontLeftTarget);
        robot.rearRightDrive.setTargetPosition(rearRightTarget);
        robot.rearLeftDrive.setTargetPosition(rearLeftTarget);

        robot.frontRightDrive.setPower(DRIVE_DISTANCE_POWER);
        robot.frontLeftDrive.setPower(DRIVE_DISTANCE_POWER);
        robot.rearRightDrive.setPower(DRIVE_DISTANCE_POWER);
        robot.rearLeftDrive.setPower(DRIVE_DISTANCE_POWER);

        while (opModeIsActive() && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
            // Update telemetry & Allow time for other processes to run
            telemetry.update();
            idle();
        }
    }

    public void driveForTime(double forward, double right, double msToRun)
    {
        telemetry.update();

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
            idle();
        }
        robot.frontRightDrive.setPower(0.0);
        robot.frontLeftDrive.setPower(0.0);
        robot.rearRightDrive.setPower(0.0);
        robot.rearLeftDrive.setPower(0.0);
    }

    public void turn(double degrees) {
        telemetry.addData("Turning ", degrees);
        telemetry.addData("encoder pos ", robot.frontRightDrive.getCurrentPosition());
        telemetry.update();

        int targetPosition = (int)(degrees * TURN_TICKS_PER_DEGREE);
        int frontRightTarget = robot.frontRightDrive.getCurrentPosition() - (int)(targetPosition);
        int frontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(targetPosition);
        int rearRightTarget = robot.rearRightDrive.getCurrentPosition() - (int)(targetPosition);
        int rearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + (int)(targetPosition);

        robot.frontRightDrive.setTargetPosition(frontRightTarget);
        robot.frontLeftDrive.setTargetPosition(frontLeftTarget);
        robot.rearRightDrive.setTargetPosition(rearRightTarget);
        robot.rearLeftDrive.setTargetPosition(rearLeftTarget);


        // TODO: Ease into movement instead of going directly to max power.
        robot.frontRightDrive.setPower(TURN_POWER);
        robot.frontLeftDrive.setPower(TURN_POWER);
        robot.rearRightDrive.setPower(TURN_POWER);
        robot.rearLeftDrive.setPower(TURN_POWER);

        while (opModeIsActive() && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
            // Update telemetry & Allow time for other processes to run
            telemetry.update();
            idle();
        }
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
        telemetry.addData("Turning To: ", degrees);
        telemetry.addData("encoder pos ", robot.frontRightDrive.getCurrentPosition());
        telemetry.update();

        // TODO: Ease into movement instead of going directly to max power.

        double error = getError(degrees);

        double turnPower = TURN_POWER;

        if (error > 0) {
            startTurningLeft(turnPower);
        } else {
            startTurningRight(turnPower);
        }
        while (opModeIsActive()) {
            // Update telemetry & Allow time for other processes to run
            error = getError(degrees);

            if (Math.abs(error) < 15 && turnPower > TURN_POWER * .2) {
                turnPower *= .95;
                robot.frontRightDrive.setPower(turnPower);
                robot.frontLeftDrive.setPower(turnPower);
                robot.rearRightDrive.setPower(turnPower);
                robot.rearLeftDrive.setPower(turnPower);
            }

            telemetry.addData("Error is: ", error);
            telemetry.update();
            if (Math.abs(error) < TURN_ERROR_THRESHOLD) {
                stopWheels();
                idle();
                break;
            } else if (error > 0) {
                startTurningLeft(turnPower);
            } else {
                startTurningRight(turnPower);
            }
            idle();
        }
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
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    interface Action {
        void execute(UltimateAuton ua, double... parameters);
    }

    class Instruction {
        public InstructionType instructionType;
        public double parameters[];

        Instruction(InstructionType it, double... parameters) {
            this.instructionType = it;
            this.parameters = parameters;
        }

        public void execute(UltimateAuton ua) {
            this.instructionType.action.execute(ua, this.parameters);
        }
    }

    enum InstructionType {
        DRIVE_FOR_TIME(new Action() {
            @Override
            public void execute(UltimateAuton ua, double... parameters) {
                ua.driveForTime(parameters[0], parameters[1], parameters[2]);
            }
        }),
        DRIVE_DISTANCE(new Action() {
            @Override
            public void execute(UltimateAuton ua, double... parameters) {
                ua.driveDistance(parameters[0], parameters[1]);
            }
        }),
        TURN(new Action() {
            @Override
            public void execute(UltimateAuton ua, double... parameters) {
                ua.turn(parameters[0]);
            }
        }),
        TURN_TO(new Action() {
            @Override
            public void execute(UltimateAuton ua, double... parameters) {
                ua.turnTo(parameters[0]);
            }
        });

        public Action action;
        InstructionType(Action action) {
            this.action = action;
        }
    }

}
