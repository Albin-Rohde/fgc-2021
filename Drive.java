package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Motor;


public class Drive {
  private Motor m1 = null;
  private Motor m2 = null;
  private Motor m3 = null;
  private Motor m4 = null;

  public Drive(Motor m1, Motor m2, Motor m3, Motor m4) {
    this.m1 = m1;
    this.m2 = m2;
    this.m3 = m3;
    this.m4 = m4;
  }

  public void forward() {
    this.m1.setPower(1);
    this.m2.setPower(1);
    this.m3.setPower(1);
    this.m4.setPower(1);
  }

  public void backward() {
    this.m1.setPower(-1);
    this.m2.setPower(-1);
    this.m3.setPower(-1);
    this.m4.setPower(-1);
  }

  public void stop() {
    this.m1.setPower(0);
    this.m2.setPower(0);
    this.m3.setPower(0);
    this.m4.setPower(0);
  }

  public void emergencyStop() {
    boolean doneWithM1 = false;
    boolean doneWithM2 = false;
    boolean doneWithM3 = false;
    boolean doneWithM4 = false;

    while(true) {
      doneWithM1 = this.m1.brake();
      doneWithM2 = this.m2.brake();
      doneWithM3 = this.m3.brake();
      doneWithM4 = this.m4.brake();
      if (doneWithM1 || doneWithM2 || doneWithM3 || doneWithM4) {
        this.m1.setPower(0);
        this.m2.setPower(0);
        this.m3.setPower(0);
        this.m4.setPower(0);
        return;
      }
    }
  }
}
