package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="ADS_RED_BACK", group="Linear OpMode")
public class ADS_RED_BACK extends LinearOpMode {
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
    public static double Start_R2_X=63.20 ;
    public static double Start_R2_Y=19.85;
    public static double Start_R2_H=180 ;
    public static double SPX=-5.90;
    public static double SPY=-5.04;
    public static double SPH=50;
    public static double P1X=-11.64;
    public static double PY=-40.20;
    public static double PY2=0;
    public static double PH=-85;
    public static double P2X=14.64;
    @Override
    public void runOpMode() {
        Pose2d Drive_from_start = new Pose2d(Start_R2_X, Start_R2_Y, Math.toRadians(Start_R2_H));
        MecanumDrive drive = new MecanumDrive(hardwareMap,Drive_from_start);
        for (int i = 0; i < 2; i++) {
            Shooter.setPower(shooter_pre_B);
            sleep(2000);
            Intake.setPower(1);
            Trigger.setPosition(1);
            sleep(500);
            Trigger.setPosition(0);
            sleep(500);
        }
        Shooter.setPower(0);
        Intake.setPower(1);
        Pose2d Start_P1 = new Pose2d(new Vector2d(Start_R2_X,Start_R2_Y), Math.toRadians(Start_R2_H));
        Actions.runBlocking(
                drive.actionBuilder(Start_P1)
                        .splineToLinearHeading(new Pose2d(36.80, 31.65, Math.toRadians(90.00)), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(36.69, 50.73), Math.toRadians(90.00))
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
            Intake.setPower(1);
            Trigger.setPosition(1);
            sleep(500);
            Trigger.setPosition(0);
            sleep(500);
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
            Intake.setPower(1);
            Trigger.setPosition(1);
            sleep(500);
            Trigger.setPosition(0);
            sleep(500);
        }
        Shooter.setPower(0);
        Actions.runBlocking(
                drive.actionBuilder(LP_SP)
                        .splineToLinearHeading(new Pose2d(P2X, PY, Math.toRadians(PH)), Math.toRadians(PH))
                        .build());


    }
}


