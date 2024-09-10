/*
 * This this contains the class that controls the robot's drive train
 */

package common;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@com.acmerobotics.dashboard.config.Config

public class Drive extends Thread {

    public static double PID_DRIVE_KP = 0.02;
    public static double PID_DRIVE_KI = 0;
    public static double PID_DRIVE_KD = 0;

    public static double DRIFT_COEFFICIENT = 0.0015;

    final boolean LOG_VERBOSE = true;

    // Drive train
    private final double WHEEL_DIAMETER_INCHES = (96 / 25.4);    // 96 mm wheels converted to inches
    //private final double COUNTS_PER_MOTOR_REV = 28 * 20;         // HD Hex Motor Encoder Ticks * gearing
    private final double COUNTS_PER_MOTOR_REV = 384.5;           // Gobilda Yellow Jacket Motor 5203-2402-0001
    private final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    private final double RAMP_DISTANCE = COUNTS_PER_INCH * 12;   // ramp down distance in encoder counts
    private final double RAMP_TIME = 1000;                       // ramp up time in milliseconds
    private final double RAMP_MIN_SPEED = 0.2;

    private final double MIN_SPEED = 0.25;
    private final double MAX_SPEED = 0.9;
    private final double MAX_ROTATE_SPEED = 0.50;

    public enum DIRECTION { FORWARD, BACK, LEFT, RIGHT, TURN_LEFT, TURN_RIGHT, DRIVER, STOOPED }

    // Color sensor
    static final float COLOR_SENSOR_GAIN = 2.2F;

    public enum COLOR {RED, BLUE}

    //  Drive train motors
    public DcMotorEx leftFrontDrive = null;   //  Used to control the left front drive wheel
    public DcMotorEx rightFrontDrive = null;  //  Used to control the right front drive wheel
    public DcMotorEx leftBackDrive = null;    //  Used to control the left back drive wheel
    public DcMotorEx rightBackDrive = null;   //  Used to control the right back drive wheel

    public DcMotorEx odometer = null;

    private Gyro gyro;
    private IMU imu;
    public double yaw = 0;

    private NormalizedColorSensor colorSensor = null;

    public DistanceSensor distanceSensor;

    private final ElapsedTime elapsedTime = new ElapsedTime();

    private PIDController pidDrive;
    private double startHeading = 0;

    private boolean running = true;
    private boolean driving = false;

    private int leftFrontStartPos  = 0;
    private int rightFrontStartPos = 0;
    private int leftBackStartPos   = 0;
    private int rightBackStartPos  = 0;

    // Sign on motor power, 1 for forward, -1 for backwards
    private int leftFrontSign  = 0;
    private int rightFrontSign = 0;
    private int leftBackSign   = 0;
    private int rightBackSign  = 0;

    private int lastPosition = 0;
    public double totalDrift = 0;

    private DIRECTION lastDirection = DIRECTION.STOOPED;

    List<DcMotorEx> motors;
    LinearOpMode opMode;

    /**
     * Contractor
     *
     * @param opMode
     */
    public Drive(LinearOpMode opMode) {
        this.opMode = opMode;
        this.setName("Drive");
        init();
    }

    /**
     * Initialize the drive train motors.
     */
    private void init() {

        initIMU();

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(PID_DRIVE_KP, PID_DRIVE_KI, PID_DRIVE_KD);

        try {
//            gyro = new Gyro(opMode.hardwareMap, "imu");
            gyro = new Gyro(opMode.hardwareMap, Config.IMU,
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        } catch (Exception e) {
            Logger.error(e, "Hardware not found");
        }

        try {
            leftFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_FRONT);
            rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_FRONT);
            leftBackDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_BACK);
            rightBackDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_BACK);

            //colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, Config.COLOR_SENSOR);
            //colorSensor.setGain(COLOR_SENSOR_GAIN);

            distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, Config.DISTANCE_SENSOR);

        } catch (Exception e) {
            Logger.error(e, "Hardware not found");
        }

        try {
            odometer = opMode.hardwareMap.get(DcMotorEx.class, Config.ODOMETER);
            odometer.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            Logger.error(e, "Hardware not found");
        }


        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = Arrays.asList(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);

         for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         }
    }

    private void initIMU () {
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    /**
     * Drive the robot with gamepad 1 joysticks one a separate thread
     */
    public void run() {

        while (!opMode.isStarted()) Thread.yield();
       Logger.message("robot drive thread started");

        ElapsedTime driveTime = new ElapsedTime();
        double lastTime = driveTime.milliseconds();
        double lastSpeed = 0;
        double accelerationPerMS = (MAX_SPEED - MIN_SPEED) / (1000 * 1.5);   // 1.5 second to accelerate to full speed
        double decelerationPerMS = (MAX_SPEED - MIN_SPEED) / (1000);     // 1 second to come to full stop

        while (running && opMode.opModeIsActive()) {

            // ToDo remove, emergency stop for testing
            if (opMode.gamepad1.back) {
                opMode.requestOpModeStop();
                break;
            }

            // Left stick to go forward back and strafe. Right stick to rotate. Left trigger accelerate.
            Gamepad gamepad = opMode.gamepad1;
            double x = -gamepad.left_stick_y / 2.0;  // Reduce drive rate to 50%.
            double y = -gamepad.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
            double yaw = -gamepad.right_stick_x / 3.0;  // Reduce rotate rate to 33%.
            double speed = (gamepad.left_trigger * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;

            // limit acceleration and deceleration to prevent skidding.
            double currentTime = driveTime.milliseconds();
            if (! driving) {
                speed = MIN_SPEED;
            } else {
                double deltaTime = currentTime - lastTime;
                double acceleration = (speed - lastSpeed) / (deltaTime);
                if ((speed > lastSpeed) && (acceleration > (accelerationPerMS * deltaTime)))
                    speed = lastSpeed + (accelerationPerMS * deltaTime);
                else if ((speed < lastSpeed) && (acceleration < decelerationPerMS * deltaTime)) {  // ToDo not currently used
                    speed = lastSpeed - (decelerationPerMS * deltaTime);
                }
            }
            lastTime = currentTime;
            lastSpeed = speed;

            if (speed > MAX_SPEED) speed = MAX_SPEED;
            if (x == 0 && y == 0 && yaw != 0) {
                if (speed > MAX_ROTATE_SPEED) speed = MAX_ROTATE_SPEED;
            }

            if (x != 0 || y != 0 || yaw != 0) {
                DIRECTION direction;
                double MAX_STICK = 0.5;

                if (yaw != 0) {
                    direction =  DIRECTION.DRIVER;
                } else if (Math.abs(x) == MAX_STICK || (x != 0 && y == 0 )) {
                    if (x > 0)
                        direction = DIRECTION.FORWARD;
                    else
                        direction = DIRECTION.BACK;
                } else if (Math.abs(y) == MAX_STICK || (x == 0 /*&& y != 0 */ )) {
                    if (y > 0)
                        direction =  DIRECTION.LEFT;
                    else
                        direction =  DIRECTION.RIGHT;
                } else {
                    direction =  DIRECTION.DRIVER;
                }

                if (direction == DIRECTION.DRIVER) {
                    moveRobot(x, y, yaw, speed);
                } else {
                    moveRobot(direction, speed);
                }

                driving = true;
                lastDirection = direction;
                //Logger.message("%-12s   %6.2f %6.2f %6.2f  %6.2f   %6.2f ", direction, x , y, yaw, gamepad.left_trigger, speed);

            } else if (driving) {
                stopRobot();
                lastDirection = DIRECTION.STOOPED;
                driving = false;

            } else {
                Thread.yield();
            }
        }
        Logger.message("robot drive thread stopped");
    }

    /**
     * Stop the thread's run method
     */
    public void end () {
        running = false;
    }

    public double getDriftCoefficient() {
        return DRIFT_COEFFICIENT;
    }

    public void setDriftCoefficient(double coefficient) {
        DRIFT_COEFFICIENT = coefficient;
    }

    // Set the sign of the motor power value for each motor. The sign determines
    // the direction of the motor.
    private void setMotorDirections (DIRECTION direction) {

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        } else {
            leftFrontSign = 0;
            rightFrontSign = 0;
            leftBackSign = 0;
            rightBackSign = 0;
        }
    }

    private double rampPower (double speed, double target) {

        int position = (
                Math.abs(leftFrontDrive.getCurrentPosition()) +
                Math.abs(rightFrontDrive.getCurrentPosition()) +
                Math.abs(leftBackDrive.getCurrentPosition()) +
                Math.abs(rightBackDrive.getCurrentPosition()) ) / 4;

        double speedRange = Math.max(Math.abs(speed) - RAMP_MIN_SPEED, 0);
        double ramUp = (elapsedTime.milliseconds() / RAMP_TIME) * speedRange + RAMP_MIN_SPEED;
        double ramDown = ((Math.abs(target) - (double) position) / RAMP_DISTANCE) * speedRange + RAMP_MIN_SPEED;
        //double ramDown = (Math.pow((Math.abs(target) - position), 2) / Math.pow(RAMP_DISTANCE, 2)) * speedRange + RAMP_MIN_SPEED;
        double rampPower = Math.min(Math.min(ramUp, ramDown), speed);
/*
        Logger.message("target %6.2f  position %d ramUp %5.2f  ramDown %5.2f  ramPower %5.2f  %5.2f  ",
                target, position, ramUp, ramDown, rampPower,
                ((Math.abs(target) - position) / RAMP_DISTANCE)
                );
*/
        return rampPower;
    }


    /**
     * Move robot according to desired axes motions
     *
     * @param x   Positive is forward
     * @param y   Positive is strafe left
     * @param yaw Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {

        double leftFrontPower  = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower   = x + y - yaw;
        double rightBackPower  = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("power %f %f %f %f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }

    public void moveRobot(double x, double y, double yaw, double speed) {

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        if (x == 0 && y == 0 && yaw == 0 ) {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;

        } else {
            leftFrontPower = x - y - yaw;
            rightFrontPower = x + y + yaw;
            leftBackPower = x + y - yaw;
            rightBackPower = x - y + yaw;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (speed == 0)
                speed = MIN_SPEED;
            else if (speed > MAX_SPEED) {
                speed = MAX_SPEED;
            }
            if (yaw != 0 && (x == 0 && y == 0)) {
                if (speed > MAX_ROTATE_SPEED)
                    speed = MAX_ROTATE_SPEED;
            }

            double scale = (1 / max) * speed;

            leftFrontPower *= scale;
            rightFrontPower *= scale;
            leftBackPower *= scale;
            rightBackPower *= scale;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("power %f %f %f %f %f", speed, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }


    public void  moveRobotWithPIDControl(DIRECTION direction, double speed) {

        // If the direction has changed get the encoder positions and motor directions
        if (direction != lastDirection) {
            setMotorDirections(direction);
            lastDirection = direction;

            // Set up parameters for driving in a straight line.
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(-PID_DRIVE_KP*3, PID_DRIVE_KP*3);   //
            pidDrive.setInputRange(-12, 12);
            pidDrive.enable();

            startHeading = getOrientation();
            lastPosition = leftFrontDrive.getCurrentPosition();
            totalDrift = 0;
        }

        int position = leftFrontDrive.getCurrentPosition();
        double heading = getOrientation();
        double angle = heading - startHeading;
        // The heading range is from -180 to 180. Check if the heading wrapped around.
        if (angle > 180)
            angle = 360 - angle;
        else if (angle < -180)
            angle = angle + 360;
        double traveled =  (double) (Math.abs(position - lastPosition)) / encoderTicksPerInch();
        double drift = traveled * Math.sin(Math.toRadians(angle));
        lastPosition = position;
        totalDrift += drift;

        opMode.telemetry.addData("Drift", "%8.2f", totalDrift);


        // Use PID with drift input to drive in a straight line.
//        double correction = pidDrive.performPID(totalDrift);
//        double speedCorrection = (speed * correction) / 100;
        double PID_MAX_OUTPUT = 0.03;
        double PID_MIN_OUTPUT = -PID_MAX_OUTPUT;
        double speedCorrection = speed * Math.max(Math.min((totalDrift*PID_DRIVE_KP), PID_MAX_OUTPUT), PID_MIN_OUTPUT);

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double error = 0.0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontPower =  speed + speedCorrection + error;
            leftBackPower =   speed + speedCorrection + error;
            rightFrontPower = speed - speedCorrection;
            rightBackPower =  speed - speedCorrection;

        } else if (direction == DIRECTION.BACK) {
            leftFrontPower =  speed - speedCorrection;
            leftBackPower =   speed - speedCorrection;
            rightFrontPower = speed + speedCorrection;
            rightBackPower =  speed + speedCorrection;

        } else if (direction == DIRECTION.RIGHT) {
            leftFrontPower =  speed + speedCorrection;
            leftBackPower =   speed - speedCorrection;
            rightFrontPower = speed - speedCorrection;
            rightBackPower =  speed + speedCorrection;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontPower =  speed - speedCorrection;
            leftBackPower =   speed + speedCorrection;
            rightFrontPower = speed + speedCorrection;
            rightBackPower =  speed - speedCorrection;

        } else {
            return;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("seconds: 4.2  power: %6.4f %6.4f %6.4f %6.4f   position: %6d %6d %6d %6d %6d    velocity: %5.2f %5.2f %5.2f %5.2f   angle: %5.2f   traveled: %4.2f   drift: %6.2f   total: %6.2f   correction: %6.3f",
                    elapsedTime.seconds(),
                    leftFrontPower,
                    rightFrontPower,
                    leftBackPower,
                    rightBackPower,
                    leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(),
                    leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition(),
                    odometer.getCurrentPosition(),
                    Math.abs(leftFrontDrive.getVelocity()) / encoderTicksPerInch(),
                    Math.abs(rightFrontDrive.getVelocity()) / encoderTicksPerInch(),
                    Math.abs(leftBackDrive.getVelocity()) / encoderTicksPerInch(),
                    Math.abs(rightBackDrive.getVelocity()) / encoderTicksPerInch(),
                    angle,
                    traveled,
                    drift,
                    totalDrift,
                    speedCorrection);
        }
    }

    /**
     *  Method to move the robot in the specified direction. The encoder are used the correct for drift
     *
     * @param direction direction to move
     * @param speed motor speed (-1 to 1)
     */
    public void moveRobot(DIRECTION direction, double speed) {

        int leftFrontSign = 0;
        int rightFrontSign = 0;
        int leftBackSign = 0;
        int rightBackSign = 0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        }

        // If the direction has changed get the encoder positions.
        if (direction != lastDirection) {
            //setMotorDirections(direction);
            leftFrontStartPos = leftFrontDrive.getCurrentPosition();
            rightFrontStartPos = rightFrontDrive.getCurrentPosition();
            leftBackStartPos = leftBackDrive.getCurrentPosition();
            rightBackStartPos = rightBackDrive.getCurrentPosition();
            lastDirection = direction;
        }

        // Correct for drift
        double leftFrontPos =  Math.abs(leftFrontDrive.getCurrentPosition()  - leftFrontStartPos);
        double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition() - rightFrontStartPos);
        double leftBackPos =   Math.abs(leftBackDrive.getCurrentPosition()   - leftBackStartPos);
        double rightBackPos =  Math.abs(rightBackDrive.getCurrentPosition()  - rightBackStartPos);
        double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

        double leftFrontAdjust = (maxPos - leftFrontPos) * DRIFT_COEFFICIENT;
        double rightFrontAdjust = (maxPos - rightFrontPos) * DRIFT_COEFFICIENT;
        double leftBackAdjust = (maxPos - leftBackPos) * DRIFT_COEFFICIENT;
        double rightBackAdjust = (maxPos - rightBackPos) * DRIFT_COEFFICIENT;

        double leftFrontPower = (speed + leftFrontAdjust) * leftFrontSign;
        double rightFrontPower = (speed + rightFrontAdjust) * rightFrontSign;
        double leftBackPower = (speed + leftBackAdjust) * leftBackSign;
        double rightBackPower = (speed + rightBackAdjust) * rightBackSign;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("moveDistance: power: %4.2f %4.2f %4.2f %4.2f    adjust: %4.3f %4.3f %4.3f %4.3f     position: %6.0f %6.0f %6.0f %6.0f",
                    leftFrontPower,
                    rightFrontPower,
                    leftBackPower,
                    rightBackPower,
                    leftFrontAdjust,
                    rightFrontAdjust,
                    leftBackAdjust,
                    rightBackAdjust,
                    leftFrontPos,
                    rightFrontPos,
                    leftBackPos,
                    rightBackPos);
        }
    }


    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     *
     * @param direction direction to move
     * @param speed motor speed (-1 to 1)
     * @param inches  distance to move in inches, positive for forward, negative for backward
     * @param timeout timeout in milliseconds, 0 for no timeout
     */
    public void moveDistance(DIRECTION direction, double speed, double inches, double timeout) {

        int leftFrontSign = 0;
        int rightFrontSign = 0;
        int leftBackSign = 0;
        int rightBackSign = 0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        }

        DcMotor.RunMode mode = leftFrontDrive.getMode();
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Determine new target position, and pass to motor controller
        int target = (int) (inches * encoderTicksPerInch());
        leftFrontDrive.setTargetPosition(target * leftFrontSign);
        rightFrontDrive.setTargetPosition(target * rightFrontSign);
        leftBackDrive.setTargetPosition(target * leftBackSign);
        rightBackDrive.setTargetPosition(target * rightBackSign);

        // Turn On RUN_TO_POSITION
        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Looping until we move the desired distance
        elapsedTime.reset();
        boolean moving = true;
        while (opMode.opModeIsActive() && moving) {

            // Correct for drift
            double leftFrontPos = Math.abs(leftFrontDrive.getCurrentPosition());
            double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition());
            double leftBackPos = Math.abs(leftBackDrive.getCurrentPosition());
            double rightBackPos = Math.abs(rightBackDrive.getCurrentPosition());
            double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

            double speedRange = Math.max(Math.abs(speed) - RAMP_MIN_SPEED, 0);
            double ramUp = (elapsedTime.milliseconds() / RAMP_TIME) * speedRange + RAMP_MIN_SPEED;
            double ramDown = (Math.pow((Math.abs(target) - maxPos), 2) / Math.pow(RAMP_DISTANCE, 2)) * speedRange + RAMP_MIN_SPEED;
            double rampPower = Math.min(Math.min(ramUp, ramDown), speed);

            double leftFrontAdjust = (maxPos - leftFrontPos) * DRIFT_COEFFICIENT;
            double rightFrontAdjust = (maxPos - rightFrontPos) * DRIFT_COEFFICIENT;
            double leftBackAdjust = (maxPos - leftBackPos) * DRIFT_COEFFICIENT;
            double rightBackAdjust = (maxPos - rightBackPos) * DRIFT_COEFFICIENT;

            leftFrontDrive.setPower((rampPower + leftFrontAdjust) * leftFrontSign);
            rightFrontDrive.setPower((rampPower + rightFrontAdjust) * rightFrontSign);
            leftBackDrive.setPower((rampPower + leftBackAdjust) * leftBackSign);
            rightBackDrive.setPower((rampPower + rightBackAdjust) * rightBackSign);

            for (DcMotor motor : motors)
                if (! motor.isBusy())
                    moving = false;

            if (timeout > 0 && elapsedTime.milliseconds() >= timeout) {
                Logger.message("moveDistance: timed out");
                break;
            }

            if (LOG_VERBOSE)
                Logger.message("power: %4.2f %4.2f %4.2f %4.2f    adjust: %4.2f %4.2f %4.2f %4.2f     position: %6d %6d %6d %6d     velocity: %4.0f %4.0f %4.0f %4.0f     heading %6.1f ",
                        leftFrontDrive.getPower(),
                        rightFrontDrive.getPower(),
                        leftBackDrive.getPower(),
                        rightBackDrive.getPower(),
                        leftFrontAdjust,
                        rightFrontAdjust,
                        leftBackAdjust,
                        rightBackAdjust,
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition(),
                        leftFrontDrive.getVelocity(),
                        rightFrontDrive.getVelocity(),
                        leftBackDrive.getVelocity(),
                        rightBackDrive.getVelocity(),
                        getOrientation());
        }

        // Stop all motion;
        for (DcMotor motor : motors)
            motor.setPower(0);

        // Restore run mode to prior state
        for (DcMotor motor : motors)
            motor.setMode(mode);

        opMode.sleep(2000);     // TODo remove

        Logger.message("%s  target  %6.2f  traveled %6.2f %6.2f %6.2f %6.2f  heading %6.2f  time %6.2f",
                direction,
                (double)target / encoderTicksPerInch(),
                (double)leftFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)leftBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                getOrientation(),
                elapsedTime.seconds());
    }

    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     *
     * @param direction direction to move
     * @param speed motor speed (-1 to 1)
     * @param inches  distance to move in inches, positive for forward, negative for backward
     * @param timeout timeout in milliseconds, 0 for no timeout
     */
    public void moveDistanceWithPIDControl(DIRECTION direction, double speed, double inches, double timeout) {

        DcMotor.RunMode mode = leftFrontDrive.getMode();
        DcMotor.ZeroPowerBehavior zeroPowerBehavior = leftFrontDrive.getZeroPowerBehavior();
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Determine new target position, and pass it to the motor controller
        setMotorDirections(direction);
        int target = (int) (inches * encoderTicksPerInch());
        leftFrontDrive.setTargetPosition(target * leftFrontSign);
        rightFrontDrive.setTargetPosition(target * rightFrontSign);
        leftBackDrive.setTargetPosition(target * leftBackSign);
        rightBackDrive.setTargetPosition(target * rightBackSign);

        // Turn On RUN_TO_POSITION
        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Looping until we move the desired distance
        elapsedTime.reset();
        boolean moving = true;
        while (opMode.opModeIsActive() && moving) {

            double rampPower = rampPower(speed, target);

            moveRobotWithPIDControl(direction, rampPower);

            for (DcMotor motor : motors)
                if (! motor.isBusy())
                    moving = false;

            if (timeout > 0 && elapsedTime.milliseconds() >= timeout) {
                Logger.message("moveDistance timed out");
                break;
            }
        }

        // Stop all motion;
        for (DcMotor motor : motors)
            motor.setPower(0);

        // ToDo testing, remove
        double velocity = leftFrontDrive.getVelocity();
        do {
            opMode.sleep(100);
        } while (velocity == leftFrontDrive.getVelocity());


        // Restore the motor to its prior state
        for (DcMotor motor : motors) {
            motor.setMode(mode);
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }

        Logger.message("%s  target  %6.2f  traveled %6.2f %6.2f %6.2f %6.2f  heading %6.2f  time %6.2f",
                direction,
                (double)target / encoderTicksPerInch(),
                (double)leftFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)leftBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                getOrientation(),
                elapsedTime.seconds());
    }


    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     *
     * @param direction direction to move
     * @param speed motor speed (-1 to 1)
     * @param inches  distance to move in inches, positive for forward, negative for backward
     * @param timeout timeout in milliseconds, 0 for no timeout
     */
    public void moveDistanceWithGyro(DIRECTION direction, double speed, double inches, double timeout) {

        // variables to set the sign of the motor power value
        int leftFrontSign = 0;
        int rightFrontSign = 0;
        int leftBackSign = 0;
        int rightBackSign = 0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        }

        DcMotor.RunMode mode = leftFrontDrive.getMode();
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Determine new target position, and pass to motor controller
        int target = (int) (inches * encoderTicksPerInch());
        leftFrontDrive.setTargetPosition(target * leftFrontSign);
        rightFrontDrive.setTargetPosition(target * rightFrontSign);
        leftBackDrive.setTargetPosition(target * leftBackSign);
        rightBackDrive.setTargetPosition(target * rightBackSign);

        // Turn On RUN_TO_POSITION
        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double lastLeftFrontPos = Math.abs(leftFrontDrive.getCurrentPosition());
        double lastRightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition());
        double lastLeftBackPos = Math.abs(leftBackDrive.getCurrentPosition());
        double lastRightBackPos = Math.abs(rightBackDrive.getCurrentPosition());
        double startHeading = getOrientation();
        double totalDrift = 0;

        // Looping until we move the desired distance
        elapsedTime.reset();
        boolean moving = true;
        while (opMode.opModeIsActive() && moving) {

            // Correct for the differance of the motor speeds
            double leftFrontPos = Math.abs(leftFrontDrive.getCurrentPosition());
            double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition());
            double leftBackPos = Math.abs(leftBackDrive.getCurrentPosition());
            double rightBackPos = Math.abs(rightBackDrive.getCurrentPosition());
            double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

            double speedRange = Math.max(Math.abs(speed) - RAMP_MIN_SPEED, 0);
            double ramUp = (elapsedTime.milliseconds() / RAMP_TIME) * speedRange + RAMP_MIN_SPEED;
            double ramDown = (Math.pow((Math.abs(target) - maxPos), 2) / Math.pow(RAMP_DISTANCE, 2)) * speedRange + RAMP_MIN_SPEED;
            double rampPower = Math.min(Math.min(ramUp, ramDown), speed);

            double leftFrontAdjust = (maxPos - leftFrontPos) * DRIFT_COEFFICIENT;
            double rightFrontAdjust = (maxPos - rightFrontPos) * DRIFT_COEFFICIENT;
            double leftBackAdjust = (maxPos - leftBackPos) * DRIFT_COEFFICIENT;
            double rightBackAdjust = (maxPos - rightBackPos) * DRIFT_COEFFICIENT;

            double heading = getOrientation();
            double traveled =  (Math.abs(leftFrontPos - lastLeftFrontPos) +
                    Math.abs(rightFrontPos - lastRightFrontPos) +
                    Math.abs(leftBackPos - lastLeftBackPos) +
                    Math.abs(rightBackPos - lastRightBackPos) ) / 2 / encoderTicksPerInch();

            double angle = heading - startHeading;
            // The heading range is from -180 to 180. Check if the heading wrapped around.
            if (angle > 180)
                angle = 360 - angle;
            else if (angle < -180)
                angle = angle + 360;
            double drift = traveled * Math.sin(Math.toRadians(angle));
            totalDrift += drift;

            double driftAdjust = Math.abs(totalDrift) * 0.0015;
            double leftFrontDriftAdjust = 0;
            double rightFrontDriftAdjust = 0;
            double leftBackDriftAdjust = 0;
            double rightBackDriftAdjust = 0;

            if (totalDrift > 0.5) {
                if (direction == DIRECTION.FORWARD) {
                    rightFrontDriftAdjust = driftAdjust;
                    rightBackDriftAdjust = driftAdjust;
                } else if (direction == DIRECTION.BACK) {
                    leftFrontDriftAdjust = driftAdjust;
                    leftBackDriftAdjust = driftAdjust;
                } else if (direction == DIRECTION.LEFT) {
                    rightFrontDriftAdjust = driftAdjust;
                    leftBackDriftAdjust = driftAdjust;
                } else if (direction == DIRECTION.RIGHT) {
                    leftFrontDriftAdjust = driftAdjust;
                    rightBackDriftAdjust = driftAdjust;
                }
            } else if (totalDrift < -0.5) {
                if (direction == DIRECTION.FORWARD) {
                    leftFrontDriftAdjust = driftAdjust;
                    leftBackDriftAdjust = driftAdjust;
                } else if (direction == DIRECTION.BACK) {
                    rightFrontDriftAdjust = driftAdjust;
                    rightBackDriftAdjust = driftAdjust;
                } else if (direction == DIRECTION.LEFT) {
                    leftFrontDriftAdjust = driftAdjust;
                    rightBackDriftAdjust = driftAdjust;
                } else if (direction == DIRECTION.RIGHT) {
                    rightFrontDriftAdjust = driftAdjust;
                    leftBackDriftAdjust = driftAdjust;
                }
            }

            lastLeftFrontPos = leftFrontPos;
            lastRightFrontPos = rightFrontPos;
            lastLeftBackPos = leftBackPos;
            lastRightBackPos = rightBackPos;

            leftFrontDrive.setPower((rampPower + leftFrontAdjust + leftFrontDriftAdjust) * leftFrontSign);
            rightFrontDrive.setPower((rampPower + rightFrontAdjust + rightFrontDriftAdjust) * rightFrontSign);
            leftBackDrive.setPower((rampPower + leftBackAdjust + leftBackDriftAdjust) * leftBackSign);
            rightBackDrive.setPower((rampPower + rightBackAdjust + rightBackDriftAdjust) * rightBackSign);

            for (DcMotor motor : motors)
                if (! motor.isBusy())
                    moving = false;

            if (timeout > 0 && elapsedTime.milliseconds() >= timeout) {
                Logger.message("moveDistance: timed out");
                break;
            }

            if (LOG_VERBOSE)
                Logger.message("power: %4.2f %4.2f %4.2f %4.2f    adjust: %6.4f %6.4f %6.4f %6.4f    position: %6d %6d %6d %6d    heading %6.2f    angle %6.2f    traveled %6.2f    drift %6.3f    adjust: %6.4f %6.4f %6.4f %6.4f",
                        leftFrontDrive.getPower(),
                        rightFrontDrive.getPower(),
                        leftBackDrive.getPower(),
                        rightBackDrive.getPower(),
                        leftFrontAdjust,
                        rightFrontAdjust,
                        leftBackAdjust,
                        rightBackAdjust,
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition(),
                        getOrientation(),
                        angle,
                        traveled,
                        drift,
                        leftFrontDriftAdjust,
                        rightFrontDriftAdjust,
                        leftBackDriftAdjust,
                        rightBackDriftAdjust);
        }

        // Stop all motion;
        for (DcMotor motor : motors)
            motor.setPower(0);

        // Restore run mode to prior state
        for (DcMotor motor : motors)
            motor.setMode(mode);

        Logger.message("%s  target  %6.2f  traveled %6.2f %6.2f %6.2f %6.2f  heading %6.2f  drift %6.2f  time %6.2f",
                direction,
                (double)target / encoderTicksPerInch(),
                (double)leftFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)leftBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                getOrientation(),
                totalDrift,
                elapsedTime.seconds());
    }


    /**
     * Move the robot until the specified color is detected.
     *
     * @param color the color to detect
     * @param x positive for forward, negative for backwards
     * @param y positive for strafe left ???, negative for strafe right ???
     * @param speed drive speed
     * @param maxInches maximum distance to travel in inches, (0 for no maximum)
     * @param timeout timeout in milliseconds (0 for no timeout)
     */
    public boolean moveToColor(COLOR color, double x, double y, double speed, double maxInches, double timeout){

        boolean found = false;
        float[] hsvValues = new float[3];
        ElapsedTime elapsedTime = new ElapsedTime();
        float lastHue = 0;
        float lastSaturation = 0;

        resetEncoders();
        moveRobot(x, y, 0, speed);
        elapsedTime.reset();

        while (! found && opMode.opModeIsActive())  {
            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to HSV color space
            Color.colorToHSV(colors.toColor(), hsvValues);
            float hue = hsvValues[0];
            float saturation = hsvValues[1];

            if (hue != lastHue || saturation != lastSaturation) {
                Logger.message("hue %f saturation %f", hue, saturation);
                lastHue = hue;
                lastSaturation = saturation;
            }
            if (color == COLOR.BLUE) {
                if (hue >= 190 && hue <= 230 && saturation >= .5) {
                    Logger.message("blue line found");
                    found = true;
                }
            } else if (color == COLOR.RED) {
                if ((hue >= 0 && hue <= 90) && saturation >= .5) {
                    Logger.message("red line found");
                    found = true;
                }
            }
            if (maxInches > 0 && (maxInches <= getDistanceTraveled())) {
                Logger.warning("no line found, traveled %5.2f inches", getDistanceTraveled());
                break;
            }
            if (timeout > 0 && elapsedTime.milliseconds() > timeout){
                Logger.warning("timeout, no line found, traveled %5.2f inches", getDistanceTraveled());
                break;
            }
        }
        stopRobot();
        return found;
    }

    /**
     * Move forward until the distance sensor detects an object at the specified distance
     *
     * @param speed   speed to move forward
     * @param inches  distance for the object to stop
     * @param timeout timeout in milliseconds
     * @return true if an object was detected at the specified distance
     */
    public boolean moveToObject (double speed, double inches, double timeout) {

        // If manually driving, exit
        if (driving) return false;

        ElapsedTime elapsedTime = new ElapsedTime();
        int count = 0;
        double startDistance = 0;
        double average;
        boolean found = false;

        resetEncoders();
        moveRobot(DIRECTION.FORWARD, speed);
        elapsedTime.reset();

        while (! found && opMode.opModeIsActive()) {
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);

            if (count == 0) {
                average = 0;
                startDistance = distance;
            } else {
                average = (startDistance - distance) / count;
                //Logger.message("distance %6.2f  %6.2f", distance, average);
            }
            count++;

            if (distance - inches <= average / 2) {
                stopRobot();
                //Logger.message("object found, distance %6.2f ", distance);
                found = true;
 //           } else if ( distance - inches < 2) {
 //               moveRobot(1, 0, 0, 0.1);
            }

            // return if timed out, of manually driving
            if (elapsedTime.milliseconds() > timeout || driving) {
                stopRobot();
                Logger.warning("no object found, traveled %6.2f inches", getDistanceTraveled());
                break;
            }
        }

        return found;
    }

    /**
     * Stop all the drive train motors.
     */
    public void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        lastDirection = DIRECTION.STOOPED;
    }

    public void setBraking(boolean enabled) {

        for (DcMotor motor : motors) {
            if (enabled)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void resetEncoders() {

        for (DcMotor motor : motors) {
            DcMotor.RunMode mode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(mode);
        }
    }

    public double encoderTicksPerInch() {
        return (COUNTS_PER_INCH * Settings.getDriveFactor());
    }

    public List<Double> getWheelPositions() {

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotor motor : motors) {
            int position = motor.getCurrentPosition();
            wheelPositions.add((double)position / encoderTicksPerInch());
        }
        return wheelPositions;
    }

    public double getDistanceTraveled() {
        double traveled = 0;

        for (DcMotor motor : motors)
            traveled += motor.getCurrentPosition();
        return traveled / motors.size() / encoderTicksPerInch();
    }

    /**
     *  Return the current orientation of the robot.
     * @return orientation in degrees in a range of -180 to 180
     */
    public double getOrientation() {
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        //return orientation.getYaw(AngleUnit.DEGREES);
        yaw = gyro.getYaw();
        return yaw;
    }

    public void resetOrientation() {
        //imu.resetYaw();
        gyro.resetYaw();
    }

    public void forward (double distance) {
        moveDistance(DIRECTION.FORWARD,.4, distance, 0);
    }

    public void back (double distance) {

        moveDistance(DIRECTION.BACK,.4, distance, 0);
    }

    public void strafeLeft (double distance) {
        //distance = Math.sqrt(Math.pow(distance,2 ) + Math.pow(distance,2));
        distance *= Settings.getStrafeFactor();
        moveDistance(DIRECTION.LEFT,.4, distance, 0);
    }

    public void strafeRight (double distance) {
        //distance = Math.sqrt(Math.pow(distance,2 ) + Math.pow(distance,2));
        distance *= Settings.getStrafeFactor();
        moveDistance(DIRECTION.RIGHT,.4, distance, 0);
    }

    public void turn(double degrees) {

        double circumference = 2 * Math.PI * Settings.getTurnFactor();
        double inches = Math.abs(degrees) / 360 * circumference;
        if (degrees > 0)
            moveDistance(DIRECTION.TURN_LEFT, 0.4,  inches, 0 );
        else
            moveDistance(DIRECTION.TURN_RIGHT, 0.4,  inches, 0 );
    }

    public void turnWithIMU(double degrees) {
        resetOrientation();
        opMode.sleep(200);
        if (degrees > 0) {
            moveRobot(0, 0, 1, 0.3);
            while (opMode.opModeIsActive()) {
                double current = getOrientation();
                Logger.message("turn: degrees %6.1f  current %6.2f", degrees, current);
                if (current >= degrees-1)
                    break;
            }
        } else if (degrees < 0) {
            moveRobot(0, 0, -1, 0.3);
            while (opMode.opModeIsActive()) {
                double current = getOrientation();
                Logger.message("turn: degrees %6.1f  current %6.2f", degrees, current);
                if (current <= degrees+1)
                    break;
            }
        }
        stopRobot();
    }

    public double distanceToObject () {
        if (distanceSensor != null)
            return distanceSensor.getDistance(DistanceUnit.INCH);

        return -1;
    }

} // end of class

