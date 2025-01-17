package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drivetrain {
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private boolean fieldCentric = false;
    private boolean debounce = false;
    public Drivetrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveRobot(Gamepad gamepad1, Gamepad gamepad2, double heading) {
        double[] inputs = getTeleopDriveInputs(gamepad1, gamepad2, heading);
        driveMovement(inputs[0], inputs[1], inputs[2], inputs[3]);
    }

    public double[] getTeleopDriveInputs(Gamepad gamepad1, Gamepad gamepad2, double heading) {
        double inputY = -gamepad1.left_stick_y;
        double inputX = gamepad1.left_stick_x;
        double inputRot = gamepad1.right_stick_x;

        //check buttons
        if(gamepad1.x && !debounce){
            fieldCentric = !fieldCentric;
        } else if (!gamepad1.x && debounce) {
            debounce = false;
        }

        if(fieldCentric){
            //rotate inputs to align with robot
            double rotX = inputX*Math.cos(-heading) - inputY*Math.sin(-heading);
            double rotY = inputX*Math.sin(-heading) + inputY*Math.cos(-heading);

            inputX = rotX * 1.1;
            inputY = rotY;
        }

        // Divisor is used to scale the other inputs if an input is too large or slow down the robot
        double divisor = Math.max(Math.abs(inputY) + Math.abs(inputX) + Math.abs(inputRot), 1) + gamepad1.right_trigger*3;
        if (gamepad2.dpad_up) {
            inputY += 0.25;
        } else if (gamepad2.dpad_down) {
            inputY -= 0.25;
        }
        if (gamepad2.right_bumper) {
            inputRot += 0.125;
        } else if (gamepad2.left_bumper) {
            inputRot -= 0.125;
        }




        return new double[] {inputY, inputX, inputRot, divisor};
    }

    public void driveMovement(double inputY, double inputX, double inputRot, double divisor) {
        backLeft.setPower((inputY - inputX + inputRot)/divisor);
        backRight.setPower((inputY + inputX - inputRot)/divisor);
        frontLeft.setPower((inputY + inputX + inputRot)/divisor);
        frontRight.setPower((inputY - inputX - inputRot)/divisor);
    }

    public boolean getFieldCentric(){
        return fieldCentric;
    }
}