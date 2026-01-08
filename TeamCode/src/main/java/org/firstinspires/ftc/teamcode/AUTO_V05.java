package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
@Config
public class AUTO_V05 extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Intake = null;
    private DcMotor Shooter = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private Servo Trigger = null;
    public static double shooter_pre_A = .8;
    public static double shooter_pre_B = .75;
    public static double shooter_pre_C = .7;
    double shooter_pre_slecter = 0;
    public static int sleep_timer=2500;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL/LO");
        backLeftDrive = hardwareMap.get(DcMotor.class, "RL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR/RO");
        backRightDrive = hardwareMap.get(DcMotor.class, "RR/BO");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Trigger = hardwareMap.get(Servo.class, "Trigger");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Trigger.setPosition(0);

        waitForStart();
        runtime.reset();
        Shooter.setPower(shooter_pre_A);
        sleep(2000);
//        Trigger.setPosition(1);
//        sleep(1000);
//        Trigger.setPosition(0);
//        sleep(1000);
//        Intake.setPower(1);
//        sleep(2000);
//        Intake.setPower(0);
//        sleep(1000);
//        Trigger.setPosition(1);
//        sleep(1000);
//        Trigger.setPosition(0);

        for (int i = 0; i < 3; i++) {
            sleep(1000);
            Trigger.setPosition(1);
            sleep(1000);
            Trigger.setPosition(0);
            sleep(1000);
            Intake.setPower(1);
            sleep(1000);
            Intake.setPower(0);
//            sleep(1000);

        }


        sleep(2000);
        Shooter.setPower(0);

        backLeftDrive.setPower(-.2);
        backRightDrive.setPower(-.2);
        frontLeftDrive.setPower(-.2);
        frontRightDrive.setPower(-.2);
        sleep(sleep_timer);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);

    }
}
