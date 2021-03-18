package org.firstinspires.ftc.teamcode;

public class MovementInstruction {
    public InstructionType instructionType;
    public double parameters[];

    public MovementInstruction(InstructionType it, double... parameters) {
        this.instructionType = it;
        this.parameters = parameters;
    }

    public void execute(MovementBehaviors behaviors) {
        this.instructionType.action.execute(behaviors, this.parameters);
    }

    interface Action {
        void execute(MovementBehaviors behaviors, double... parameters);
    }

    public enum InstructionType {
        FIRE_RING(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.fireTrigger(parameters[0]);
            }
        }),

        WAIT_FOR_TIME(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.waitForTimeInMilliseconds(parameters[0]);
            }
        }),

        TURN_ON_CONVEYOR(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.turnOnCoveyor(parameters[0]);
            }
        }),

        TURN_OFF_CONVEYOR(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.turnOffCoveyor();
            }
        }),

        TURN_ON_SHOOTER(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.turnOnShooter(parameters[0]);
            }
        }),

        TURN_OFF_SHOOTER(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.turnOffShooter();
            }
        }),

        DRIVE_FOR_TIME(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.driveForTime(parameters[0], parameters[1], parameters[2]);
            }
        }),

        DRIVE_DISTANCE(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.driveDistance(parameters[0], parameters[1]);
            }
        }),

        TURN(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.turn(parameters[0]);
            }
        }),

        TURN_TO(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.turnTo(parameters[0]);
            }
        }),

        MOVE_WOBBLE_ARM_TO_POSITION(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.moveWobbleArmToPosition(parameters[0], parameters[1]);
            }
        }),

        START_DRIVING(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.startDriving(parameters[0]);
            }
        }),
        STOP_WHEELS(new Action() {
            @Override
            public void execute(MovementBehaviors behaviors, double... parameters) {
                behaviors.stopWheels();
            }
        });

        public Action action;
        InstructionType(Action action) {
            this.action = action;
        }
    }
}

