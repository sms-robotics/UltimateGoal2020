package org.firstinspires.ftc.teamcode;

class Instruction {
    public InstructionType instructionType;
    public double parameters[];

    Instruction(InstructionType it, double... parameters) {
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

