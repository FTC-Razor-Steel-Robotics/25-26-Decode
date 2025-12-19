package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp_v1", group="Linear OpMode")
@Config
public class TeleOp_V1 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Intake = null;
    private Servo Trigger = null;
    public static double shooter_pre_A = .8;
    public static double shooter_pre_B = .75;
    public static double shooter_pre_C = .7;
    double shooter_pre_slecter = 0;

	Robot robot = new Robot();


    @Override
    public void runOpMode() {
		robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        Intake=hardwareMap.get(DcMotor.class,"Intake");
        Trigger=hardwareMap.get(Servo.class,"Trigger");

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Trigger.setPosition(0);

        waitForStart();
        runtime.reset();

        boolean right_bumper_prev = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
			robot.driveMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

			robot.spinShooter(gamepad2.left_bumper, gamepad2.right_bumper && !right_bumper_prev);
			right_bumper_prev = gamepad2.right_bumper;

            if (gamepad2.right_trigger>.2){
                Trigger.setPosition(1);
            }else {
                Trigger.setPosition(0);
            }

            Intake.setPower(-gamepad2.left_stick_y);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
    }}
