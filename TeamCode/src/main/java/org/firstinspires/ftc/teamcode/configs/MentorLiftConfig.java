package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MentorLiftConfig {
	public static boolean liftEnabled = false;

	public static String frontLiftString = "frontLift";
	public static String rearLiftString = "rearLift";

	public static boolean frontLiftReverse = true;
	public static boolean rearLiftReverse = true;

	public static double liftSpeed = 0.3;
	public static int liftMax = 500;
}
