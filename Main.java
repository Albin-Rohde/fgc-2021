package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Main", group="Linear Opmode")
public class Main extends LinearOpMode {
  private boolean running = false;

  private Drive getDriver() {
    DcMotor m1 = null;
    DcMotor m2 = null;
    DcMotor m3 = null;
    DcMotor m4 = null;
    Servo s1 = null;
    //DistanceSensor sensor = null;

    m1 = hardwareMap.get(DcMotor.class, "1");
    m2 = hardwareMap.get(DcMotor.class, "2");
    m3 = hardwareMap.get(DcMotor.class, "3");
    m4 = hardwareMap.get(DcMotor.class, "4");
    s1 = hardwareMap.get(Servo.class, "steer");
    //sensor = hardWareMap.get(DistanceSensor.class, "rs");

    Motor motor1 = new Motor(m1, true); //left back
    Motor motor2 = new Motor(m2, false); //right back
    Motor motor3 = new Motor(m3, true); //left front
    Motor motor4 = new Motor(m4, false); //right front
    return new Drive(motor1, motor2, motor3, motor4, s1);
  }

  @Override
  public void runOpMode() {
    Drive driver = getDriver();
    driver.turnStraight();
    waitForStart();
    this.running = true;
    while (opModeIsActive() && this.running) {
      double turn = gamepad1.left_stick_x;
      if (gamepad1.dpad_left){
        driver.turnLeft();
      } else if (gamepad1.dpad_right) {
        driver.turnRight();
      } else {
        driver.turnStraight();
      }
      if (gamepad1.a) {
        driver.forward();
      } else if (gamepad1.b) {
        driver.power(-1);
      } else {
        driver.power(0);
      }

      telemetry.update();
      //driver.power(speed);
      //driver.turn(turn);
      /*
      driver.forward();
      driver.sleep(60);
      driver.turnLeft();
      driver.sleep(75);
      driver.turnStraight();
      driver.sleep(220);
      driver.turnRight();
      driver.sleep(60);
      driver.turnStraight();
      driver.sleep(29);
      driver.turnRight();
      driver.sleep(65);
      driver.turnStraight();
      driver.sleep(65);
      driver.turnRight();
      driver.sleep(40);
      driver.turnStraight();
      driver.sleep(120);
      driver.stop();
      /*
      driver.turnRight();
      driver.sleep(65);
      driver.turnStraight();
      driver.sleep(45);
      driver.turnRight();
      driver.sleep(60);
      driver.stop();
      */
      //driver.turnRight();
      //driver.sleep(30);
      //driver.sleep(20);
      //driver.stop();
      /*
      sleep(600);
      driver.turnStraight();
      sleep(1100);
      driver.turnRight();
      sleep(400);
      driver.turnStraight();
      sleep(400);
      driver.turnRight();
      sleep(1200);
      driver.turnStraight();
      sleep(1200);
      driver.stop();
      this.running = false;
      */
      //sleep(5000);
      //this.running = true;
    }
  }
}
