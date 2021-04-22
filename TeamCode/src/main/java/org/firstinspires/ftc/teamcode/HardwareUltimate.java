package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
    private DcMotor arm = null;
    private TouchSensor touchUp = null;
    private TouchSensor touchDown = null;

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

        try
        {
            touchUp = this.hardwareMap.get(TouchSensor.class, "wobbleup");
        }
        catch (Exception p_exception) { };

        try
        {
            touchDown = this.hardwareMap.get(TouchSensor.class, "wobbledown");
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

        // Initialize the PID Calculators
        movementBehaviors.initializeXPIDCalculator(0.0025, 0.0, 0.0, false);
        movementBehaviors.initializeYPIDCalculator(0.0025, 0.0, 0.0,false);
        movementBehaviors.initializeZPIDCalculator(0.015, 0.000, 0.0,false);
        movementBehaviors.initializeTurnPIDCalculator(0.0075, 0.000, 0.0,false);

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

    public ActionWobbleArm createAndInitializeWobbleArm(LinearOpMode opMode) {
        ActionWobbleArm actionWobbleArm = new ActionWobbleArm(arm, opMode, touchUp, touchDown);
        actionWobbleArm.initialize();
        return actionWobbleArm;
    }

    public ActionTrigger createAndInitializeTrigger() {
        ActionTrigger actionTrigger = new ActionTrigger(trigger);
        actionTrigger.initialize();
        return actionTrigger;
    }

    public SoundManager createAndInitializeSoundManager() {
        SoundManager soundManager = new SoundManager(this.hardwareMap);
        soundManager.initialize();
        return soundManager;
    }
}

