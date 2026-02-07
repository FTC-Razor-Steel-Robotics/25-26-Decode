package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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
    public static double shooter_pre_A = 0.875 * 12.5;
    double shooter_pre_slecter = 0;
    VoltageSensor voltageSensor;
    public static int sleep_timer=1000;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        //Initialize our voltage sensor
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL/LO");
        backLeftDrive = hardwareMap.get(DcMotor.class, "RL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backRightDrive = hardwareMap.get(DcMotor.class, "RR");
        Shooter = hardwareMap.get(DcMotor.class,"Shooter/BO");
        Intake = hardwareMap.get(DcMotor.class,"Intake/RO");
        Trigger = hardwareMap.get(Servo.class,"Trigger");


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
        sleep(8000);
        runtime.reset();
        Shooter.setPower(shooter_pre_A / voltageSensor.getVoltage());
        sleep(5000);
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
            Shooter.setPower(shooter_pre_A / voltageSensor.getVoltage());
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


        sleep(0000);
        Shooter.setPower(0);

        backLeftDrive.setPower(.5);
        backRightDrive.setPower(-.5);
        frontLeftDrive.setPower(-.5);
        frontRightDrive.setPower(-.5);
        sleep(sleep_timer);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);

    }
}
