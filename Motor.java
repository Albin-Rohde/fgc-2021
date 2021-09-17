package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.stream.Stream;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class Motor {
  private DcMotor m;
  private double speedMultiplier = TeamConfig.driveMultiplier;
  private boolean reversed;
  public double motorPower;


  public Motor(DcMotor m, boolean reversed) {
    this.m = m;
    this.reversed = reversed;
  }

  public void setPower(double pwr) {
    this.motorPower = pwr;
    double powerSignal = 0;
    if (this.reversed) {
      powerSignal = (pwr * this.speedMultiplier) * -1;
    } else {
      powerSignal = (pwr * this.speedMultiplier);
    }
    this.m.setPower(powerSignal);
  }


  public int getCurrentPosition() {
    return this.m.getCurrentPosition();
  }

  public void stop() {
    this.m.setPower(0);
  }
}
