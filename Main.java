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

@TeleOp(name="TeleOp1", group="Linear Opmode")
public class TeleOp1 extends LinearOpMode {
  // utils
  private ElapsedTime runtime = new ElapsedTime();

  // Motors
  private DcMotor motor1 = null;
  private DcMotor motor2 = null;

  private void mapDevices() {
    motor1 = hardwareMap.get(DcMotor.class, "1");
    motor2 = hardwareMap.get(DcMotor.class, "2");
  }

  private void drive() {
    motor1.setPower(-1);
    motor2.setPower(1);
  }

  private void stopMotor() {
    motor1.setPower(0);
    motor2.setPower(0);
  }
  /*
    The base motor supplies 27 positions per round
    with a level 3 gear that is 27 * 3 positions. 81 positions
   */
  int gearLevel = 3;
  int posIncremt = 27 * gearLevel;
  int posPerRound = 27;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    mapDevices();


    runtime.reset();
    waitForStart();
    int beginPos = motor1.getCurrentPosition();
    int laps = 0;
    while (opModeIsActive()) {
      drive();
      if (motor1.getCurrentPosition() >= beginPos + 81) {
        beginPos += 81;
        telemetry.addData("info", "time" + runtime);
        laps++;
        runtime.reset();
      }
      if (laps == 10) {
        stopMotor();
        telemetry.update();
        sleep(1000000);
      }
    }
  }
}
