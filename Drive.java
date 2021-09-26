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
  private boolean running = true;
  public Motor m1 = null;
  public Motor m2 = null;
  public Motor m3 = null;
  public Motor m4 = null;
  private Servo s1 = null;
  private DistanceSensor sensor = null;
  private Thread t1 = null; // t1: thread that checks the sensor and initiates emergency brake


  public Drive(
    Motor m1,
    Motor m2,
    Motor m3,
    Motor m4,
    Servo s1
    //DistanceSensor sensor
  ) {
    this.m1 = m1;
    this.m2 = m2;
    this.m3 = m3;
    this.m4 = m4;
    this.s1 = s1;
    this.s1.setPosition(0.5);
    //this.sensor = sensor;
  }

  public void forward() {
    if (this.running) {
      this.m1.setPower(1);
      this.m2.setPower(1);
      this.m3.setPower(1);
      this.m4.setPower(1);
    }
  }

  public void power(double pw) {
    this.m1.setPower(pw);
    this.m2.setPower(pw);
    this.m3.setPower(pw);
    this.m4.setPower(pw);
  }

  public void backward() {
    if (this.running) {
      this.m1.setPower(-1);
      this.m2.setPower(-1);
      this.m3.setPower(-1);
      this.m4.setPower(-1);
    }
  }

  public void stop() {
    if (this.running) {
      this.m1.setPower(0);
      this.m2.setPower(0);
      this.m3.setPower(0);
      this.m4.setPower(0);
      this.running = false;
    }
  }

  public void turn(double pos) {
    this.s1.setPosition(pos);
  }

  public void turnLeft() {
    if (this.running) {
      this.s1.setPosition(0);
    }
  }

  public void turnRight() {
    if (this.running) {
      this.s1.setPosition(1);
    }
  }

  public void turnStraight() {
    if (this.running) {
      this.s1.setPosition(0.5);
    }
  }

  public void sleep(int ticks) {
    int initialTick = this.m2.getCurrentPosition();
    while(true) {
      int currentTick = this.m2.getCurrentPosition();
      if (currentTick >= initialTick + ticks) {
        return;
      }
    }
  }

  public void emergencyStop() {
    this.running = false;
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
