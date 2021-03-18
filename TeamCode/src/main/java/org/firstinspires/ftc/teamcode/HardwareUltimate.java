package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;

// This class is NOT an opmode, it is used to define all the specific hardware for a single robot.

public class HardwareUltimate
{
    /* DriveTrain */
    public DcMotor frontLeftDrive  = null;
    public DcMotor frontRightDrive = null;
    public DcMotor rearLeftDrive   = null;
    public DcMotor rearRightDrive  = null;

    private DcMotor shooter  = null;
    private DcMotor conveyor = null;
    private DcMotor arm;

    /* Manipulator */

//    public DcMotor armShoulder = null;
//    public DcMotor armGrab = null;
//    public Servo servoCapStone = null;
//    public Servo servoFound1 = null;
//    public Servo servoFound2 = null;
//    public Servo servoFound3 = null;
//    public Servo servoArm1 = null;
//    public Servo servoArm3 = null;

    /* Auton */
    public Servo sensorAxis = null;
    public NormalizedColorSensor colorSensor = null;
    public DistanceSensor sensorRange = null; // used for 2m
    public DistanceSensor colorRange = null; // used for REV Distance+Color sensor
    public BNO055IMU imu = null;

    public String teamID = "";
    private HardwareMap hardwareMap;
    private Servo trigger;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean Auton) {
        this.hardwareMap = ahwMap;

        if (Auton == true) {
            // Initialize the bot-specific distance sensor
            try
            {
                sensorRange = ahwMap.get(DistanceSensor.class, "2m10644");
                teamID = "10644";
            }
            catch (Exception p_exception) { };

            try
            {
                sensorRange = ahwMap.get(DistanceSensor.class, "2m10645");
                teamID = "10645";
            }
            catch (Exception p_exception) { };

            // 2m Color Sensor Servo
            try
            {
                sensorAxis = ahwMap.get(Servo.class, "2maxis");
            }
            catch (Exception p_exception) { };
        }

        // Define and Initialize Motors
        try
        {
            frontRightDrive = ahwMap.get(DcMotor.class, "fr");
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD) ;
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setTargetPosition(0);}
        catch (Exception p_exception) { };

        try
        {
            rearRightDrive = ahwMap.get(DcMotor.class, "rr");
            rearRightDrive.setDirection(DcMotor.Direction.FORWARD) ;
            rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRightDrive.setTargetPosition(0);
        }
        catch (Exception p_exception) { };

        try
        {
            frontLeftDrive = ahwMap.get(DcMotor.class, "fl");
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE) ;
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftDrive.setTargetPosition(0);
        }
        catch (Exception p_exception) { };

        try
        {
            rearLeftDrive = ahwMap.get(DcMotor.class, "rl");
            rearLeftDrive.setDirection(DcMotor.Direction.REVERSE) ;
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeftDrive.setTargetPosition(0);
        }
        catch (Exception p_exception) { };

        try
        {
            shooter = ahwMap.get(DcMotor.class, "out");
        }
        catch (Exception p_exception) { };

        try
        {
            conveyor = ahwMap.get(DcMotor.class, "in");
        }
        catch (Exception p_exception) { };

        try
        {
            arm = ahwMap.get(DcMotor.class, "wobble");
        }
        catch (Exception p_exception) { };

        try
        {
            trigger = ahwMap.get(Servo.class, "fire");
        }
        catch (Exception p_exception) { };


        try
        {
            imu = this.hardwareMap.get(BNO055IMU.class, "imu");
        }
        catch (Exception p_exception) { };

    }

    public SensorIMU createAndInitializeIMU(LinearOpMode opMode) {
        SensorIMU sensorIMU = new SensorIMU(imu);
        sensorIMU.initialize(opMode);

        return sensorIMU;
    }

    public VisionManager createAndInitializeVisionManager() {
        VisionManager visionManager = new VisionManager();

        visionManager.initialize(hardwareMap);

        return visionManager;
    }

    public MovementBehaviors createAndInitializeMovementBehaviors(LinearOpMode opMode, ActionConveyor conveyor, ActionShooter shooter, ActionTrigger trigger, ActionWobbleArm wobbleArm, SensorIMU sensorIMU) {
        MovementBehaviors movementBehaviors = new MovementBehaviors(opMode, this, conveyor, shooter, trigger, wobbleArm, sensorIMU);
        return movementBehaviors;
    }

    public VisionWebcamScanner createAndInitializeWebcamScanner() {
        VisionWebcamScanner webcamScanner = new VisionWebcamScanner();
        webcamScanner.initialize(this.hardwareMap);
        return webcamScanner;
    }

    public ActionConveyor createAndInitializeConveyor() {
        ActionConveyor actionConveyor = new ActionConveyor(conveyor);
        actionConveyor.initialize();
        return actionConveyor;
    }

    public ActionShooter createAndInitializeShooter() {
        ActionShooter actionShooter = new ActionShooter(shooter);
        actionShooter.initialize();
        return actionShooter;
    }

    public ActionWobbleArm createAndInitializeWobbleArm() {
        ActionWobbleArm actionWobbleArm = new ActionWobbleArm(arm);
        actionWobbleArm.initialize();
        return actionWobbleArm;
    }

    public ActionTrigger createAndInitializeTrigger() {
        ActionTrigger actionTrigger = new ActionTrigger(trigger);
        actionTrigger.initialize();
        return actionTrigger;
    }
}

