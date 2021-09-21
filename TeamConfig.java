package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class TeamConfig {
  private int gearLevel = 1;
  private int posPerMotorRound = 27;

  public final static int posPerRound = 27 * 1; // 27 is the motor rpm, 27 * gear level is the wheel rpm
  public final static double driveMultiplier = 0.15;
}
