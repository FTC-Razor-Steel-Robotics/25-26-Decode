package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Disabled
@Autonomous(name="ADS_BLUE_FRONT", group="Linear OpMode")
public class ADS_BLUE_FRONT extends LinearOpMode {
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
    public static double Start_B1_X=-49.86 ;
    public static double Start_B1_Y=-51.83;
    public static double Start_B1_H=55 ;
    public static double SPX=-5.90;
    public static double SPY=-5.04;
    public static double SPH=50;
    public static double P1X=-11.64;
    public static double PY=-40.20;
    public static double PH=-85;
    public static double P2X=14.64;


    @Override
    public void runOpMode() {
        Pose2d Drive_from_start = new Pose2d(Start_B1_X, Start_B1_Y, Math.toRadians(Start_B1_H));
        MecanumDrive drive = new MecanumDrive(hardwareMap,Drive_from_start);
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(Drive_from_start)
                        .splineToConstantHeading(new Vector2d(SPX, SPY), Math.toRadians(SPH))
                        .build());
        Pose2d LP_P1 = new Pose2d(new Vector2d(SPX,SPY), Math.toRadians(SPH));
        for (int i = 0; i < 2; i++) {
            Shooter.setPower(shooter_pre_B);
            sleep(2000);
            Trigger.setPosition(1);
            sleep(500);
            Trigger.setPosition(0);
            sleep(500);
            Intake.setPower(1);
            sleep(500);
            Intake.setPower(0);
        }
        Shooter.setPower(0);
        Intake.setPower(1);
        Actions.runBlocking(
                drive.actionBuilder(LP_P1)
                        .splineToLinearHeading(new Pose2d(P1X, SPY, Math.toRadians(PH)), Math.toRadians(PH))
                        .splineToConstantHeading(new Vector2d(P1X, PY), Math.toRadians(PH))
                        .build());
        Pose2d P1_LP = new Pose2d(new Vector2d(P1X,PY), Math.toRadians(PH));
        Intake.setPower(0);
        sleep(500);
        Actions.runBlocking(
                drive.actionBuilder(P1_LP)
                        .splineToConstantHeading(new Vector2d(P1X/2,SPY),Math.toRadians(PH))
                        .splineToLinearHeading(new Pose2d(SPX,SPY, Math.toRadians(SPH)),Math.toRadians(SPH))
                        .build());
        for (int i = 0; i < 2; i++) {
            Shooter.setPower(shooter_pre_B);
            sleep(2000);
            Trigger.setPosition(1);
            sleep(500);
            Trigger.setPosition(0);
            sleep(500);
            Intake.setPower(1);
            sleep(500);
            Intake.setPower(0);
        }
        Shooter.setPower(0);
        Intake.setPower(1);

        Pose2d LP_P2 = new Pose2d(new Vector2d(SPX,SPY), Math.toRadians(SPH));
        Actions.runBlocking(
                drive.actionBuilder(LP_P2)
                        .splineToLinearHeading(new Pose2d(P2X, SPY, Math.toRadians(PH)), Math.toRadians(PH))
                        .splineToConstantHeading(new Vector2d(P2X, PY), Math.toRadians(PH))
                        .build());
        Pose2d P2_LP = new Pose2d(new Vector2d(P2X,PY), Math.toRadians(PH));
        sleep(500);
        Intake.setPower(0);
        Actions.runBlocking(
                drive.actionBuilder(P2_LP)
                        .splineToConstantHeading(new Vector2d(P2X/2,SPY),Math.toRadians(PH))
                        .splineToLinearHeading(new Pose2d(SPX,SPY, Math.toRadians(SPH)),Math.toRadians(SPH))
                        .build());
        Pose2d LP_SP = new Pose2d(new Vector2d(SPX,SPY),Math.toRadians(SPH));
        for (int i = 0; i < 2; i++) {
            Shooter.setPower(shooter_pre_B);
            sleep(2000);
            Trigger.setPosition(1);
            sleep(500);
            Trigger.setPosition(0);
            sleep(500);
            Intake.setPower(1);
            sleep(500);
            Intake.setPower(0);
        }
        Shooter.setPower(0);
        Actions.runBlocking(
                drive.actionBuilder(LP_SP)
                        .splineToLinearHeading(new Pose2d(P2X, PY, Math.toRadians(PH)), Math.toRadians(PH))
                        .build());








    }
}


