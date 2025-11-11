package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonomousIrrelevant extends LinearOpMode {
    private DcMotorEx fl_Wheel = null;
    private DcMotorEx bl_Wheel = null;
    private DcMotorEx fr_Wheel = null;
    private DcMotorEx br_Wheel = null;
    private DcMotorEx launch_motor = null;
    private CRServo left_servo = null;
    private CRServo right_servo = null;

    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     DRIVE_GEAR_REDUCTION    = 19.2;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 104 * 3.14;

    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    final double FEED_TIME_SECONDS = 0.15; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = -1.0;

    final double FAST_LAUNCHER_TARGET_VELOCITY = 2000;
    final double FAST_LAUNCHER_MIN_VELOCITY = 1990;
    final double SLOW_LAUNCHER_TARGET_VELOCITY = 1475;
    final double SLOW_LAUNCHER_MIN_VELOCITY = 1450;
    double LAUNCHER_TARGET_VELOCITY = 2000;
    double LAUNCHER_MIN_VELOCITY = 1800;

    ElapsedTime feederTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        fl_Wheel = hardwareMap.get(DcMotorEx.class, "fl_motor");
        bl_Wheel = hardwareMap.get(DcMotorEx.class, "bl_motor");
        fr_Wheel = hardwareMap.get(DcMotorEx.class, "fr_motor");
        br_Wheel = hardwareMap.get(DcMotorEx.class, "br_motor");
        launch_motor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        left_servo = hardwareMap.get(CRServo.class, "left_servo");
        right_servo = hardwareMap.get(CRServo.class, "right_servo");

        fl_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launch_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr_Wheel.setDirection(DcMotor.Direction.FORWARD);
        fl_Wheel.setDirection(DcMotor.Direction.REVERSE);
        br_Wheel.setDirection(DcMotor.Direction.FORWARD);
        bl_Wheel.setDirection(DcMotor.Direction.REVERSE);
        launch_motor.setDirection(DcMotor.Direction.REVERSE);
        left_servo.setDirection(DcMotorSimple.Direction.REVERSE);


        // moves 2 ft forward
        // for teleops, instead of 610, we'll put inside whileloop and
        // move based on input received from gamepad
        int flTarget = (int)(750 * COUNTS_PER_MM);
        int blTarget = (int)(750 * COUNTS_PER_MM);
        int frTarget = (int)(750 * COUNTS_PER_MM);
        int brTarget = (int)(750 * COUNTS_PER_MM);
        double TPS = ((double) 150 / 60) * COUNTS_PER_WHEEL_REV;

        waitForStart();

        double fl_Position = fl_Wheel.getCurrentPosition();
        double bl_Position = bl_Wheel.getCurrentPosition();
        double fr_Position = fr_Wheel.getCurrentPosition();
        double br_Position = br_Wheel.getCurrentPosition();
        double launch_Position = launch_motor.getCurrentPosition();

        telemetry.addData("Front Left Wheel Pos", fl_Position);
        telemetry.addData("Back Left Wheel Pos", bl_Position);
        telemetry.addData("Front Right Wheel Pos", fr_Position);
        telemetry.addData("Back Right Wheel Pos", br_Position);
        telemetry.addData("Launch Motor Pos", launch_Position);

        telemetry.update();

        launchState = LaunchState.IDLE;

        //shoot artifacts
        right_servo.setPower(FULL_SPEED);
        left_servo.setPower(FULL_SPEED);


        /*
        launch_motor.setVelocity(LAUNCHER_TARGET_VELOCITY);
        while (launch_motor.getVelocity() < LAUNCHER_MIN_VELOCITY) {
            sleep(1);
        }
        left_servo.setPower(FULL_SPEED);
        right_servo.setPower(FULL_SPEED);
        feederTimer.reset();
        while (feederTimer.seconds() < FEED_TIME_SECONDS) {
            sleep(1);
        }
        right_servo.setPower(STOP_SPEED);
        left_servo.setPower(STOP_SPEED);
        */



        sleep(5000);





        /*
        launch();
        sleep(5000);

        launch();
        sleep(5000);
        */

        fl_Wheel.setTargetPosition(flTarget);
        bl_Wheel.setTargetPosition(blTarget);
        fr_Wheel.setTargetPosition(frTarget);
        br_Wheel.setTargetPosition(brTarget);

        fl_Wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_Wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr_Wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_Wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl_Wheel.setVelocity(TPS);
        bl_Wheel.setVelocity(TPS);
        fr_Wheel.setVelocity(TPS);
        br_Wheel.setVelocity(TPS);

        sleep(5000);

        fl_Position = fl_Wheel.getCurrentPosition();
        bl_Position = bl_Wheel.getCurrentPosition();
        fr_Position = fr_Wheel.getCurrentPosition();
        br_Position = br_Wheel.getCurrentPosition();
        launch_Position = launch_motor.getCurrentPosition();

        telemetry.addData("Front Left Wheel Pos", fl_Position);
        telemetry.addData("Back Left Wheel Pos", bl_Position);
        telemetry.addData("Front Right Wheel Pos", fr_Position);
        telemetry.addData("Back Right Wheel Pos", br_Position);
        telemetry.addData("Launch Motor Pos", launch_Position);

        telemetry.update();

        sleep(5000);

        //while (opModeIsActive() && (leftmotor.isBusy() && rightmotor.isBusy())) {
        }

        void launch() {
            switch (launchState) {
                case IDLE:
                    launchState = LaunchState.SPIN_UP;
                    break;
                case SPIN_UP:
                    launch_motor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    if (launch_motor.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                        launchState = LaunchState.LAUNCH;
                    }
                    break;
                case LAUNCH:
                    telemetry.addData("made it to servo code?", "yes");
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
