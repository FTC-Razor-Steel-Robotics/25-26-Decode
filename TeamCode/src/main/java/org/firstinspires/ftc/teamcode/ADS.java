package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="ADS", group="Linear OpMode")
public class ADS extends LinearOpMode {
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
    public static double Start_B2_X=0 ;
    public static double Start_B2_Y=0 ;
    public static double Start_B2_H=0 ;
    @Override
    public void runOpMode() {

    }
}


