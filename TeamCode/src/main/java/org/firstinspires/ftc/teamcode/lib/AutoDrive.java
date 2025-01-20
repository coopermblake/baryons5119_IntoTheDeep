package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static double STRAFE_EFFICIENCY_FACTOR = 0.90;
    static final double     STRAFE_COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI * STRAFE_EFFICIENCY_FACTOR)); //change to the right factor

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.03;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.0001;     // Larger is more responsive, but also less stable.
    static final double     P_STRAFE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    //Initializing custom PID instances
    private CustomPID pidX = new CustomPID(0, 0, 0); // Adjust coefficients
    private CustomPID pidY = new CustomPID(0, 0, 0); // Adjust coefficients
    //kP = 0.05, kl = 0.01, kD = 0.01 - setting to zero to make sure driveforward, strafe, and drivediagonal work


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




    // Reverse movement is obtained by setting a negative distance (not speed)
    public void driveStraight(double speed, double distance, double heading) {
        double targetTicksY = distance * COUNTS_PER_INCH;
        pidY.updateController(); // Reset PID controller

        while (Math.abs(frontLeft.getCurrentPosition() - targetTicksY) > 10) {
            // Get current encoder values for Y direction
            double currentY = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 2.0;

            // Compute PID output for straight driving
            double powerY = pidY.cycleController(targetTicksY, currentY) * speed;

            // Add heading correction
            double headingCorrection = getSteeringCorrection(heading, P_TURN_GAIN);

            // Apply motor powers
            double frontLeftPower = Range.clip(powerY + headingCorrection, -1.0, 1.0);
            double frontRightPower = Range.clip(powerY - headingCorrection, -1.0, 1.0);
            double backLeftPower = Range.clip(powerY + headingCorrection, -1.0, 1.0);
            double backRightPower = Range.clip(powerY - headingCorrection, -1.0, 1.0);

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Telemetry for debugging
            telemetry.addData("Drive Straight", "Target Y: %.2f, Current Y: %.2f", targetTicksY, currentY);
            telemetry.addData("Power Y", powerY);
            telemetry.addData("Heading Correction", headingCorrection);
            telemetry.update();
        }

        // Stop motors
        moveRobot(0, 0, 0);
    }

    public void driveStrafe(double speed, double distance, double heading) {
        double targetTicksX = distance * STRAFE_COUNTS_PER_INCH;
        pidX.updateController(); // Reset PID controller

        while (Math.abs((frontLeft.getCurrentPosition() - backLeft.getCurrentPosition()) / 2.0 - targetTicksX) > 10) {
            // Get current encoder values for X direction
            double currentX = (frontLeft.getCurrentPosition() - backLeft.getCurrentPosition()) / 2.0;

            // Compute PID output for strafing
            double powerX = pidX.cycleController(targetTicksX, currentX) * speed;

            // Add heading correction
            double headingCorrection = getSteeringCorrection(heading, P_TURN_GAIN);

            // Apply motor powers
            double frontLeftPower = Range.clip(powerX + headingCorrection, -1.0, 1.0);
            double frontRightPower = Range.clip(-powerX - headingCorrection, -1.0, 1.0);
            double backLeftPower = Range.clip(-powerX + headingCorrection, -1.0, 1.0);
            double backRightPower = Range.clip(powerX - headingCorrection, -1.0, 1.0);

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Telemetry for debugging
            telemetry.addData("Drive Strafe", "Target X: %.2f, Current X: %.2f", targetTicksX, currentX);
            telemetry.addData("Power X", powerX);
            telemetry.addData("Heading Correction", headingCorrection);
            telemetry.update();
        }

        // Stop motors
        moveRobot(0, 0, 0);
    }

    public void driveDiagonal(double speed, double distanceX, double distanceY, double timeoutSeconds) {
        double targetTicksX = distanceX * STRAFE_COUNTS_PER_INCH;
        double targetTicksY = distanceY * COUNTS_PER_INCH;

        pidX.updateController(); // Reset PID controllers
        pidY.updateController();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.seconds() < timeoutSeconds) {
            // Get current encoder values
            double currentX = (frontLeft.getCurrentPosition() - frontRight.getCurrentPosition()) / 2.0;
            double currentY = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 2.0;

            // Compute PID outputs for diagonal driving
            double powerX = pidX.cycleController(targetTicksX, currentX) * speed;
            double powerY = pidY.cycleController(targetTicksY, currentY) * speed;

            // Apply motor powers
            double frontLeftPower = Range.clip(powerY + powerX, -1.0, 1.0);
            double backLeftPower = Range.clip(powerY - powerX, -1.0, 1.0);
            double frontRightPower = Range.clip(powerY - powerX, -1.0, 1.0);
            double backRightPower = Range.clip(powerY + powerX, -1.0, 1.0);

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Telemetry for debugging
            telemetry.addData("Diagonal Move", "Target X: %.2f, Target Y: %.2f", targetTicksX, targetTicksY);
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Power X", powerX);
            telemetry.addData("Power Y", powerY);
            telemetry.update();

            // Exit loop if both errors are within acceptable ranges
            if (Math.abs(targetTicksX - currentX) < 10 && Math.abs(targetTicksY - currentY) < 10) {
                break;
            }
        }

        // Stop motors
        moveRobot(0, 0, 0);
    }

    public void turnToHeading(double speed, double heading) {
        pidX.updateController(); // Reuse X PID for simplicity

        while (Math.abs(getHeading() - heading) > HEADING_THRESHOLD) {
            // Get current heading
            double currentHeading = getHeading();

            // Compute PID output for turning
            double turnPower = pidX.cycleController(heading, currentHeading) * speed;

            // Apply turning power
            moveRobot(0, turnPower, 0);

            // Telemetry for debugging
            telemetry.addData("Turn To Heading", "Target: %.2f, Current: %.2f", heading, currentHeading);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }

        // Stop motors
        moveRobot(0, 0, 0);
    }

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

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
                           //90

        // Determine the heading current error
        headingError = targetHeading - getHeading();

                       //90-0

        // Normalize the error to be within +/- 180 degrees
        headingError = normalizeHeading(headingError);

        // Calculate the turning power using proportional gain
        double turnPower = -headingError * proportionalGain;

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