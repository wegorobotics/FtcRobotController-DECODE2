package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")
//@Disabled
public class StarterBotTeleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.15; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = -1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */

    final double FAST_LAUNCHER_TARGET_VELOCITY = 1975;
    final double FAST_LAUNCHER_MIN_VELOCITY = 1950;
    final double SLOW_LAUNCHER_TARGET_VELOCITY = 1475;
    final double SLOW_LAUNCHER_MIN_VELOCITY = 1450;
    double LAUNCHER_TARGET_VELOCITY = 1475;
    double LAUNCHER_MIN_VELOCITY = 1450;

    // Declare OpMode members.
    private DcMotor fl_Wheel = null;
    private DcMotor bl_Wheel = null;
    private DcMotor fr_Wheel = null;
    private DcMotor br_Wheel = null;
    private DcMotorEx launch_motor = null;
    private CRServo left_servo = null;
    private CRServo right_servo = null;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        fl_Wheel = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_Wheel = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_Wheel = hardwareMap.get(DcMotor.class, "fr_motor");
        br_Wheel = hardwareMap.get(DcMotor.class, "br_motor");
        launch_motor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        left_servo = hardwareMap.get(CRServo.class, "left_servo");
        right_servo = hardwareMap.get(CRServo.class, "right_servo");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        fr_Wheel.setDirection(DcMotor.Direction.FORWARD);
        fl_Wheel.setDirection(DcMotor.Direction.FORWARD);
        br_Wheel.setDirection(DcMotor.Direction.FORWARD);
        bl_Wheel.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launch_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch_motor.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        left_servo.setPower(STOP_SPEED);
        right_servo.setPower(STOP_SPEED);

        launch_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        left_servo.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        //arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.dpad_up) {
            LAUNCHER_TARGET_VELOCITY = FAST_LAUNCHER_TARGET_VELOCITY;
            LAUNCHER_MIN_VELOCITY = FAST_LAUNCHER_MIN_VELOCITY;
        } else if (gamepad1.dpad_down) {
            LAUNCHER_TARGET_VELOCITY = SLOW_LAUNCHER_TARGET_VELOCITY;
            LAUNCHER_MIN_VELOCITY = SLOW_LAUNCHER_MIN_VELOCITY;
        }

        if (gamepad1.b) { // stop flywheel
            launch_motor.setVelocity(STOP_SPEED);
        }

        double launch_Position = launch_motor.getCurrentPosition();
        telemetry.addData("launchmotor", launch_Position);

        /*
         * Now we call our "Launch" function.
         */

        launch(gamepad1.rightBumperWasPressed());

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

        double joystick_direction = (-1 * Math.atan2(left_y, left_x)) + (Math.PI / 2);
        double joystick_magnitude = Math.sqrt((left_x * left_x) + (left_y * left_y));

        double left_x2 = gamepad2.left_stick_x / 2;
        double left_y2 = gamepad2.left_stick_y / 2;
        double joystick_turn2 = gamepad2.right_stick_x / 2;
        double joystick_direction2 = -1 * Math.atan2(left_y2, left_x2) / 2;
        double joystick_magnitude2 = Math.sqrt((left_x2 * left_x2) + (left_y2 * left_y2)) / 2;

        fr_Wheel.setPower(-1 * Math.sin((joystick_direction + joystick_direction2) + (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) - (joystick_turn + joystick_turn2) / 2);
        br_Wheel.setPower(1 * Math.sin((joystick_direction + joystick_direction2) - (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) - (joystick_turn + joystick_turn2) / 2);
        fl_Wheel.setPower(1 * Math.sin((joystick_direction + joystick_direction2) - (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) + (joystick_turn + joystick_turn2) / 2);
        bl_Wheel.setPower(-1 * Math.sin((joystick_direction + joystick_direction2) + (0.25 * Math.PI)) * (joystick_magnitude + joystick_magnitude2) + (joystick_turn + joystick_turn2) / 2);

        /*
         * Show the state and motor powers
         */

        telemetry.addData("State", launchState);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("motorSpeed", launch_motor.getVelocity());
        telemetry.addData("did the code push? ", "yes!");

        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*
    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
    */

    void launch(boolean shotRequested) {
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
        telemetry.update();
    }
}