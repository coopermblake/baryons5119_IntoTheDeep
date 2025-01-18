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

package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Locale;

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


public class AutoDrive {

    private final Telemetry telemetry;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;

    private final IMU imu;
    private double headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;

    //will need to remove leftSpeed, rightSpeed, leftTarget, rightTarget and add these on
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
    static final double     WHEEL_DIAMETER_INCHES   = 4.09 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     STRAFE_COUNTS_PER_INCH = 0; //FIND IRL

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.
    static final double     P_STRAFE_GAIN           = 0.001;     // Larger is more responsive, but also less stable.

    public AutoDrive(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, IMU imu, Telemetry telemetry){

        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.imu = imu;
        this.telemetry = telemetry;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed-loop speed control, and reset the heading
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();
    }
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {



            // Determine new target position for all four motors
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRight.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeft.getCurrentPosition() + moveCounts;
            backRightTarget = backRight.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontLeft.setTargetPosition(frontLeftTarget);
            frontRight.setTargetPosition(frontRightTarget);
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0, 0);

            // Keep looping while we are still active, and all motors are running
            while ((frontLeft.isBusy() && frontRight.isBusy() &&
                            backLeft.isBusy() && backRight.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
                //strafeSpeed = getSteeringCorrection(heading, P_STRAFE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(maxDriveSpeed, turnSpeed, 0);

                // Display drive status for the driver.
                sendTelemetry(true, false);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0,0);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void driveStrafe(double maxStrafeSpeed, double distance, double heading){
        int moveCounts = (int)(distance * STRAFE_COUNTS_PER_INCH);
        frontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
        frontRightTarget = frontRight.getCurrentPosition() - moveCounts;
        backLeftTarget = backLeft.getCurrentPosition() - moveCounts;
        backRightTarget = backRight.getCurrentPosition() + moveCounts;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((frontLeft.isBusy() && frontRight.isBusy() &&
                backLeft.isBusy() && backRight.isBusy())) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // if driving left, the motor correction also needs to be reversed
            if (distance < 0) turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(0, turnSpeed, maxStrafeSpeed);

            // Display drive status for the driver.
            sendTelemetry(true, false);
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0,0);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        while ((Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the turn speed to the maximum permitted value
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction to all motors
            moveRobot(0, turnSpeed,0); // 0 for no forward/backward motion

            // Display drive status for the driver
            sendTelemetry(false,false);
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
        while ((holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Apply turning correction to all four motors
            moveRobot(0, turnSpeed,0); // No forward/backward or strafing, only turning

            // Display drive status for the driver
            sendTelemetry(false,false);
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
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the four wheel motors.
     * @param drive forward/reverse motor speed
     * @param turn  clockwise/counterclockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn, double strafe) {
        driveSpeed = drive;     // Save this value as a class member for telemetry
        turnSpeed  = turn;      // Save this value as a class member for telemetry
        strafeSpeed = strafe;   // Save this value as a class member for telemetry

        // Calculate motor powers for mecanum drive
        frontLeftSpeed = drive + strafe + turn;
        frontRightSpeed = drive - strafe - turn;
        backLeftSpeed = drive - strafe + turn;
        backRightSpeed = drive + strafe - turn;

        // Scale motor powers if any exceed +/- 1.0
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
        frontLeft.setPower(frontLeftSpeed);
        frontRight.setPower(frontRightSpeed);
        backLeft.setPower(backLeftSpeed);
        backRight.setPower(backRightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    //Todo: make this argument simpler with enum
    public void sendTelemetry(boolean straight, boolean strafe) {

        telemetry.addData("Heading - Target : Current:", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error : Steer Pwr:", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData( "Wheel Speeds FL:FR:BL:BR:", " %5.2f : %5.2f : %5.2f : %5.2f",
                frontLeft.getPower(), frontRight.getPower(),
                backLeft.getPower(), backRight.getPower());

        if (straight) {
            telemetry.addData("Motion", "Straight");
            telemetry.addData("Target Pos FL:FR:BL:BR:", " %7d:%7d:%7d:%7d",
                    frontLeftTarget, frontRightTarget,
                    backLeftTarget, backRightTarget);
            telemetry.addData("Actual Pos FL:FR:BL:BR:", " %7d:%7d:%7d:%7d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        }
        else if(strafe){
            telemetry.addData("Motion", "Strafe");
            telemetry.addData("Target Pos FL:FR:BL:BR:", " %7d:%7d:%7d:%7d",
                    frontLeftTarget, frontRightTarget,
                    backLeftTarget, backRightTarget);
            telemetry.addData("Actual Pos FL:FR:BL:BR:", " %7d:%7d:%7d:%7d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        }
        else {
            telemetry.addData("Motion", "Turning");
        }

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
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            return orientation.getYaw(AngleUnit.DEGREES);
        } catch (Exception e) {
            // Handle any errors (e.g., if the IMU is not initialized)
//            telemetry.addData("IMU Error", "Unable to retrieve heading: " + e.getMessage());
//            telemetry.update();
            return 0.0; // Default to 0 degrees if IMU data is unavailable
        }
    }
}