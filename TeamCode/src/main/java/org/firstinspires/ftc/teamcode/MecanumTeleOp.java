package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name= "MecanumTeleOp")
//@Disabled
public class MecanumTeleOp extends OpMode {
    // motors
    DcMotor fl_Wheel;
    DcMotor bl_Wheel;
    DcMotor fr_Wheel;
    DcMotor br_Wheel;
    DcMotor launch_motor;
    CRServo left_servo;
    CRServo right_servo;

    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    ElapsedTime feederTimer = new ElapsedTime();

    // state machine: launch state determines state of launcher
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    @Override
    public void init() {
        // initial launcher state
        launchState = LaunchState.IDLE;

        // define motor & servos
        fl_Wheel = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_Wheel = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_Wheel = hardwareMap.get(DcMotor.class, "fr_motor");
        br_Wheel = hardwareMap.get(DcMotor.class, "br_motor");
        launch_motor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        left_servo = hardwareMap.get(CRServo.class, "left_servo");
        right_servo = hardwareMap.get(CRServo.class, "right_servo");

        fr_Wheel.setDirection(DcMotor.Direction.REVERSE);
        fl_Wheel.setDirection(DcMotor.Direction.FORWARD);
        br_Wheel.setDirection(DcMotor.Direction.REVERSE);
        bl_Wheel.setDirection(DcMotor.Direction.REVERSE);

        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_servo.setPower(STOP_SPEED);
        right_servo.setPower(STOP_SPEED);

        //launch_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        left_servo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);
    }

    boolean initialize = true;

    public void loop() {

        if (initialize) {
            initialize = false;
        }

        // encoders
        double fl_Position = fl_Wheel.getCurrentPosition();
        double bl_Position = bl_Wheel.getCurrentPosition();
        double fr_Position = fr_Wheel.getCurrentPosition();
        double br_Position = br_Wheel.getCurrentPosition();
        double launch_Position = launch_motor.getCurrentPosition();
        /* double right_Position = right_servo.getPosition();
        double left_Position = left_servo.getPosition(); */

        telemetry.addData("Front Left Wheel Pos", fl_Position);
        telemetry.addData("Back Left Wheel Pos", bl_Position);
        telemetry.addData("Front Right Wheel Pos", fr_Position);
        telemetry.addData("Back Right Wheel Pos", br_Position);
        telemetry.addData("Launch Motor Pos", launch_Position);
        /* telemetry.addData("Right Servo Pos", right_Position);
        telemetry.addData("Left Servo Pos", left_Position); */

        // wheel movement
        double left_x = gamepad1.left_stick_x;
        double left_y = gamepad1.left_stick_y;
        double joystick_turn = gamepad1.right_stick_x;
        if (gamepad1.left_stick_button) {
            left_x = (gamepad1.left_stick_x / 2);
            left_y = (gamepad1.left_stick_y / 2);
        }
        if (gamepad1.right_stick_button) {
            joystick_turn = (gamepad1.right_stick_x / 2);
        }

        double joystick_direction = -1 * Math.atan2(left_y, left_x);
        double joystick_magnitude = Math.sqrt((left_x * left_x) + (left_y * left_y));

        double left_x2 = gamepad2.left_stick_x / 2;
        double left_y2 = gamepad2.left_stick_y / 2;
        double joystick_turn2 = gamepad2.right_stick_x / 2;
        double joystick_direction2 = -1 * Math.atan2(left_y2, left_x2) / 2;
        double joystick_magnitude2 = Math.sqrt((left_x2 * left_x2) + (left_y2 * left_y2)) / 2;

        // setting power of wheels based on joystick data
        /*
        fr_Wheel.setPower((-1 * Math.sin(joystick_direction + (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        fl_Wheel.setPower((1 * Math.sin(joystick_direction + (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        br_Wheel.setPower((-1 * Math.sin(joystick_direction - (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        bl_Wheel.setPower((1 * Math.sin(joystick_direction - (0.25 * Math.PI)) * joystick_magnitude + joystick_turn) / 2);
        */

        fr_Wheel.setPower(1.024 * (-1 * Math.sin((joystick_direction + joystick_direction2) - (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) + (joystick_turn + joystick_turn2)) / 2);
        br_Wheel.setPower(1.000 * (1 * Math.sin((joystick_direction + joystick_direction2) + (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) - (joystick_turn + joystick_turn2)) / 2);
        fl_Wheel.setPower(1.094 * (-1 * Math.sin((joystick_direction + joystick_direction2) + (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) - (joystick_turn + joystick_turn2)) / 2);
        bl_Wheel.setPower(0.989 * (1 * Math.sin((joystick_direction + joystick_direction2) - (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) + (joystick_turn + joystick_turn2)) / 2);

        // launching
        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        /*
        if (gamepad1.y) {
            launch_motor.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) { // stop flywheel
            launch_motor.setVelocity(STOP_SPEED);
        }

        /*
         * Now we call our "Launch" function.
         */
        //launch(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        //telemetry.addData("motorSpeed", launch_motor.getVelocity());

        telemetry.update();
    }
}
    /*
    public void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launch_motor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launch_motor.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                left_servo.setPower(FULL_SPEED);
                right_servo.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    left_servo.setPower(STOP_SPEED);
                    right_servo.setPower(STOP_SPEED);
                }
                break;
        }
    }
}
}
*/
//bleh