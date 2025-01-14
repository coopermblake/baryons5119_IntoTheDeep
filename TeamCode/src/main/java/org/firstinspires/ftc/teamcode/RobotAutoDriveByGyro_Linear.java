/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (robot.imu) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal robot.imu interface so it will work with either the BNO055, or BHI260 robot.imu.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver Station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="EncoderBasedAuto", group="Competition Opmodes")
public class RobotAutoDriveByGyro_Linear extends LinearOpMode {
    Robot robot;
    private double headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double strafeSpeed   = 0;
    private double frontLeftSpeed = 0;
    private double frontRightSpeed = 0;
    private double backLeftSpeed = 0;
    private double backRightSpeed = 0;
    private int frontLeftTarget = 0;
    private int frontRightTarget = 0;
    private int backLeftTarget = 0;
    private int backRightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;   // need to make sure this is correct for 435 rpm yellow jacket motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.09449 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.6;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.005;     // Larger is more responsive, but also less stable.
    static final double P_STRAFE_GAIN = 0.005;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("startInit");
        telemetry.update();
        sleep(1000);
        robot = new Robot(hardwareMap, gamepad1, gamepad2);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value while waiting)
        telemetry.addLine("inInit");
        telemetry.update();
        sleep(5000);

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        telemetry.addLine("endInit");
        telemetry.update();
        // Set the encoders for closed-loop speed control, and reset the heading
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.imu.resetYaw();

        // Step through each leg of the path
        // Notes: Reverse movement is obtained by setting a negative distance (not speed)
        //        holdHeading() is used after turns to let the heading stabilize

//       driveStraight(DRIVE_SPEED, 24.0, 0.0);    // Drive Forward 24"
//        turnToHeading(TURN_SPEED, -45.0);               // Turn CW to -45 Degrees
//        holdHeading(TURN_SPEED, -45.0, 0.5);   // Hold -45 Deg heading for 1/2 second
//
//        driveStraight(DRIVE_SPEED, 17.0, -45.0);  // Drive Forward 17" at -45 degrees
//        turnToHeading(TURN_SPEED, 45.0);               // Turn CCW to 45 Degrees
//        holdHeading(TURN_SPEED, 45.0, 0.5);    // Hold 45 Deg heading for 1/2 second
//
//        driveStraight(DRIVE_SPEED, 17.0, 45.0);  // Drive Forward 17" at 45 degrees
//        turnToHeading(TURN_SPEED, 0.0);               // Turn CW to 0 Degrees
//        holdHeading(TURN_SPEED, 0.0, 1.0);    // Hold 0 Deg heading for 1 second
//
//        driveStraight(DRIVE_SPEED, -48.0, 0.0);    // Drive in Reverse 48" (should return to starting position)

        telemetry.addLine("ready to drive");
        telemetry.update();
        sleep(1000);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message

        while(opModeIsActive()) {

        }
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading,
                              double strafeDistance) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position for all four motors, including strafing
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            int strafeCounts = (int) (strafeDistance * COUNTS_PER_INCH);

            // Combine forward/reverse and strafing movements for each motor
            frontLeftTarget = robot.frontLeft.getCurrentPosition() + moveCounts + strafeCounts;
            frontRightTarget = robot.frontRight.getCurrentPosition() + moveCounts - strafeCounts;
            backLeftTarget = robot.backLeft.getCurrentPosition() + moveCounts - strafeCounts;
            backRightTarget = robot.backRight.getCurrentPosition() + moveCounts + strafeCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.frontLeft.setTargetPosition(frontLeftTarget);
            robot.frontRight.setTargetPosition(frontRightTarget);
            robot.backLeft.setTargetPosition(backLeftTarget);
            robot.backRight.setTargetPosition(backRightTarget);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0, 0); // Include strafeSpeed as a parameter

            // Keep looping while we are still active, and all motors are running
            while (opModeIsActive() &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
                            robot.backLeft.isBusy() && robot.backRight.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
                strafeSpeed = getSteeringCorrection(heading, P_STRAFE_GAIN); // Optional strafing correction

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;

                // Apply the turning and strafing corrections to the current driving speed
                moveRobot(maxDriveSpeed, turnSpeed, strafeSpeed);

                // Display drive status for the driver
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, 0);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // Keep looping while OpMode is active and the heading error is greater than the threshold
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the turn speed to the maximum permitted value
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction to all motors
            moveRobot(0, turnSpeed,0);

            // Display drive status for the driver
            sendTelemetry(false);
        }

        // Stop all motion
        moveRobot(0, 0,0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // Keep looping while OpMode is active and we have time remaining
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Apply turning correction to all four motors
            moveRobot(0, turnSpeed,0); // only turning

            // Display drive status for the driver
            sendTelemetry(false);
        }

        // Stop all motion
        moveRobot(0, 0,0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to the required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        headingError = normalizeHeading(headingError);

        // Calculate the turning power using proportional gain
        double turnPower = headingError * proportionalGain;

        // Clip the turning power to ensure it stays within the range [-1, 1]
        return Range.clip(turnPower, -1.0, 1.0);
    }

    /**
     * Normalize a heading error to be within the range [-180, 180] degrees.
     *
     * @param error  The heading error in degrees.
     * @return       The normalized heading error.
     */
    private double normalizeHeading(double error) {
        if (error > 180) {
            error -= 360;
        } else if (error <= -180) {
            error += 360;
        }
        return error;
    }

    /**
     * Move the robot with separate forward/reverse, turning, and strafing speeds.
     *
     * @param drive Forward/reverse motor speed.
     * @param turn Clockwise/counterclockwise turning motor speed.
     * @param strafe Left/right strafing motor speed.
     */
    public void moveRobot(double drive, double turn, double strafe) {
        driveSpeed = drive;
        turnSpeed = turn;
        strafeSpeed = strafe;

        // Calculate motor powers for mecanum drive
        frontLeftSpeed = drive + turn + strafe;
        frontRightSpeed = drive - turn - strafe;
        backLeftSpeed = drive + turn - strafe;
        backRightSpeed = drive - turn + strafe;

        double maxPower = Math.max(Math.abs(frontLeftSpeed),
                Math.max(Math.abs(frontRightSpeed),
                        Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed))));
        if (maxPower > 1.0) {
            frontLeftSpeed /= maxPower;
            frontRightSpeed /= maxPower;
            backLeftSpeed /= maxPower;
            backRightSpeed /= maxPower;
        }

        // Apply the calculated powers to the motors
        robot.frontLeft.setPower(frontLeftSpeed);
        robot.frontRight.setPower(frontRightSpeed);
        robot.backLeft.setPower(backLeftSpeed);
        robot.backRight.setPower(backRightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos FL:FR:BL:BR", "%7d:%7d:%7d:%7d",
                    frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
            telemetry.addData("Actual Pos FL:FR:BL:BR", "%7d:%7d:%7d:%7d",
                    robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading - Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds FL:FR:BL:BR", "%5.2f : %5.2f : %5.2f : %5.2f",
                robot.frontLeft.getPower(), robot.frontRight.getPower(),
                robot.backLeft.getPower(), robot.backRight.getPower());
        telemetry.update();
    }

    /**
     * Reads the robot's heading directly from the IMU in degrees.
     *
     * @return The current yaw angle (heading) in degrees. Returns 0 if IMU data is unavailable.
     */
    public double getHeading() {
        try {
            // Retrieve the yaw (heading) angle from the IMU
            YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
            return orientation.getYaw(AngleUnit.DEGREES);
        } catch (Exception e) {
            // Handle any errors (e.g., if the IMU is not initialized)
            telemetry.addData("IMU Error", "Unable to retrieve heading: " + e.getMessage());
            telemetry.update();
            return 0.0; // Default to 0 degrees if IMU data is unavailable
        }
    }

    //Strafe method
    public void strafe(double maxStrafeSpeed, double distance, double heading) {
        if (opModeIsActive()) {

            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = robot.frontLeft.getCurrentPosition() - moveCounts;
            frontRightTarget = robot.frontRight.getCurrentPosition() + moveCounts;
            backLeftTarget = robot.backLeft.getCurrentPosition() + moveCounts;
            backRightTarget = robot.backRight.getCurrentPosition() - moveCounts;

            robot.frontLeft.setTargetPosition(frontLeftTarget);
            robot.frontRight.setTargetPosition(frontRightTarget);
            robot.backLeft.setTargetPosition(backLeftTarget);
            robot.backRight.setTargetPosition(backRightTarget);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxStrafeSpeed = Math.abs(maxStrafeSpeed);
            moveRobot(0, 0, maxStrafeSpeed);

            while (opModeIsActive() &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
                            robot.backLeft.isBusy() && robot.backRight.isBusy())) {

                double correction = getSteeringCorrection(heading, P_DRIVE_GAIN);
                moveRobot(0, correction, maxStrafeSpeed);

                sendTelemetry(true);
            }

            moveRobot(0, 0, 0);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //Diagonal driving method
    public void driveDiagonal(double speed, double distance, double angle) {
        if (opModeIsActive()) {
            double angleRad = Math.toRadians(angle);

            double xPower = Math.cos(angleRad); // Sideways component
            double yPower = Math.sin(angleRad); // Forward component

            double normalizationFactor = Math.max(Math.abs(xPower), Math.abs(yPower));
            xPower /= normalizationFactor;
            yPower /= normalizationFactor;

            double frontLeftPower = (yPower + xPower) * speed;
            double frontRightPower = (yPower - xPower) * speed;
            double backLeftPower = (yPower - xPower) * speed;
            double backRightPower = (yPower + xPower) * speed;

            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            frontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (frontLeftPower * moveCounts);
            frontRightTarget = robot.frontRight.getCurrentPosition() + (int) (frontRightPower * moveCounts);
            backLeftTarget = robot.backLeft.getCurrentPosition() + (int) (backLeftPower * moveCounts);
            backRightTarget = robot.backRight.getCurrentPosition() + (int) (backRightPower * moveCounts);

            robot.frontLeft.setTargetPosition(frontLeftTarget);
            robot.frontRight.setTargetPosition(frontRightTarget);
            robot.backLeft.setTargetPosition(backLeftTarget);
            robot.backRight.setTargetPosition(backRightTarget);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontLeft.setPower(Math.abs(frontLeftPower));
            robot.frontRight.setPower(Math.abs(frontRightPower));
            robot.backLeft.setPower(Math.abs(backLeftPower));
            robot.backRight.setPower(Math.abs(backRightPower));

            while (opModeIsActive() &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
                            robot.backLeft.isBusy() && robot.backRight.isBusy())) {
                telemetry.addData("Diagonal Move", "Angle: %.2f, Distance: %.2f", angle, distance);
                telemetry.update();
            }

            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /*
        public driveAndRotate(IMU imu, double speed, double heading, double rotationSpeed, double rotationAngle, int timeoutMS) {
	boolean pathComplete = false;
	boolean rotationComplete = false;
        int distanceTravelled = 0;
        int initialFLticks = robot.frontLeft.getCurrentPosition();
        int initialBLticks = robot.backLeft.getCurrentPosition();
        int initialFRticks = robot.frontRight.getCurrentPosition();
        int initialBRticks = robot.backRight.getCurrentPosition();
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double xDist = targetx - currentx) + cos(targetangle - botHeading);
        double yDist = targety - currenty) + sin(targetangle - botHeading);
        double targetXTicks = xDist * COUNTS_PER_INCH * 1.1 + initialFLticks;
        double targetYTicks = yDist * COUNTS_PER_INCH + initialFLTicks;

        while (opModeIsActive() && !pathComplete && !rotationComplete) {

            double rx = rotationSpeed;

            // Have we rotated enough?
	    botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (Math.abs(rotationAngle - botHeading) < 0.1) {
                // Mark rotation complete and set rotation speed to 0
                rotationComplete = true;
                rx = 0;
            }


            // Have we translated enough?
            currentFLticks = robot.frontLeft.getCurrentPosition();
            currentBLticks = robot.backLeft.getCurrentPosition();
            currentFRticks = robot.frontRight.getCurrentPosition();
            currentBRticks = robot.backRight.getCurrentPosition();

            if ((Math.abs(xDist) >  Math.abs(yDist) && Math.abs(currentFLticks - targetXTicks) < 20) {
                 translationComplete = true;
                 speed = 0;
	    }
            if (Math.abs(yDist) > Math.abs(xDist) && Math.abs(currentFLticks - targetTicks) < 20) {
                 translationComplete = true;
                 speed = 0;
            }

            double x = speed * Math.cos(heading);
	    double y = speed * Math.sin(heading);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }

     */
}