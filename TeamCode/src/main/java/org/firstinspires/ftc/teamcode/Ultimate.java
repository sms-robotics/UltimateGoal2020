package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UltimateHardware;

@TeleOp(name="Ultimate", group="Pushbot")
public class Ultimate extends LinearOpMode {

    /* Declare OpMode members. */
    UltimateHardware robot = new UltimateHardware();   // Use a Pushbot's hardware
    float[] hsvValues = new float[3];
    final float values[] = hsvValues;
    float armNominalPowerUp = 0.6f;
    float armNominalPowerDown = 0.3f;
    float armGrabHold = 0.0f;
    float clawNominalPower = 0.2f;
    float driveNominalPower = 0.4f;
    float armOffPower = 0.0f;
    double posCapStone = 0.7d;
    float armGrab;
    int amPos;
    int aePos;
    int aeOffset;
    boolean previousDPD = false;
    boolean previousDPU = false;
    boolean previousDPL = false;
    boolean previousDPR = false;
    double foundation = 0.1;
    
    int currentPosition = 4;
    int singleDriver = 0;
    
    @Override
    public void runOpMode() {

        double[] posArray;
        posArray = new double[10];
        posArray[0] = 0.0; // this is the pre-start bot setup
        posArray[1] = 0.35; // collect stone from ground level
        posArray[2] = 0.0; // deliver stone level 1
        posArray[3] = 0.35; // deliver stone level 2
        posArray[4] = 0.1; // deliver stone level 3
        posArray[5] = 0.6; // deliver stone level 4
        posArray[6] = 0.4; // deliver stone level 5
        posArray[7] = 0.0; // deliver stone level 6
        posArray[8]=  0.4; // need 10644 arm servo
        posArray[9]=  0.05;
        
        //posArray[10]=60;
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, false);
        //move to hardware code and only run in AUTON

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        float powerReducer = 0.5f;
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            if (gamepad1.a && (singleDriver == 0)) {singleDriver = 1;}
            if (gamepad1.b && (singleDriver == 1)) {singleDriver = 2;}
            if (gamepad1.x && (singleDriver ==2)) {singleDriver = 3;}
            if (gamepad1.y && (singleDriver ==3)) {singleDriver = 4;}

            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float FrontLeft = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
            float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float BackLeft = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;

            float gamepad2LeftY = -gamepad2.left_stick_y;
            float gamepad2RightY = -gamepad2.right_stick_y;


            // clip the right/left values so that the values never exceed +/- 1
            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);

            powerReducer = driveNominalPower;
            if (gamepad1.right_trigger > 0) {
                powerReducer = 1.0f;
            }
            if (gamepad1.left_trigger > 0) {
                powerReducer = 0.1f;
            }

            // write the values to the motors
            if (robot.frontRightDrive != null) {
                robot.frontRightDrive.setPower(FrontRight * powerReducer);
            }
            if (robot.frontLeftDrive != null) {
                robot.frontLeftDrive.setPower(FrontLeft * powerReducer);
            }
            if (robot.rearLeftDrive != null) {
                robot.rearLeftDrive.setPower(BackLeft * powerReducer);
            }
            if (robot.rearRightDrive != null) {
                robot.rearRightDrive.setPower(BackRight * powerReducer);
            }


            //foundation = Range.clip(foundation, 0.0, 1.0);
            //if (robot.servoArm3 != null) {robot.servoArm3.setPosition(foundation);}
            
            //print out motor values
            telemetry.addLine()
                    .addData("front right", FrontRight)
                    .addData("front left", FrontLeft)
                    .addData("back left", BackLeft)
                    .addData("back right", BackRight);
                    //.addData("position index", currentPosition)
                    //.addData("target", posArray[currentPosition])

            telemetry.update();
        }
    }
}
