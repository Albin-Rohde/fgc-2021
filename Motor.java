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
  private boolean reversed;
  private double motorPower;
  private int brakeStartPos;
  private int maxBrakeDelta;
  private boolean brakeStarted = false;

  public Motor(DcMotor m, boolean reversed) {
    this.m = m;
    this.reversed = reversed;
  }

  public void setPower(double pwr) {
    this.motorPower = pwr;
    double powerSignal = 0;
    if (this.reversed) {
      powerSignal = (pwr * TeamConfig.driveMultiplier) * -1;
    } else {
      powerSignal = (pwr * TeamConfig.driveMultiplier);
    }
    this.m.setPower(powerSignal);
  }

  public boolean brake () {
    if (this.motorPower == 0) {
      this.setPower(0);
      return true;
    }
    if (!brakeStarted) {
      this.brakeStartPos = this.m.getCurrentPosition();
      this.maxBrakeDelta = 0;
      this.brakeStarted = true;
      this.m.setPower(((this.motorPower / Math.abs(this.motorPower)) * -1) * 0.3);
      return false;
    }
    int currentDelta = Math.abs(this.m.getCurrentPosition() - this.brakeStartPos);
    if (currentDelta > maxBrakeDelta) {
      this.maxBrakeDelta = currentDelta;
    }
    if (maxBrakeDelta - 3 > currentDelta) {
      this.setPower(0);
      return true;
    }
    return false;
  }

  public int getCurrentPosition() {
    return this.m.getCurrentPosition();
  }
}
