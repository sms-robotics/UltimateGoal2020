package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class ActionConveyor {
    private final DcMotor conveyorMotor;

    public ActionConveyor(DcMotor conveyorMotor) {
        this.conveyorMotor = conveyorMotor;
    }

    public void initialize() {
        conveyorMotor.setDirection(DcMotor.Direction.REVERSE);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnOnAtPower(double power) {
        double clippedPower = Range.clip(power, 0.0, 1.0);
        conveyorMotor.setPower(clippedPower);
    }

    public void turnOn() {
        turnOnAtPower(1.0);
    }

    public void turnOff() {
        conveyorMotor.setPower(0);
    }
}
