package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import java.lang.reflect.Field;

public class VariableFactory extends SubsystemBase {

    // 12V Auto Variables
    private final double moveTo1stBasketDuration1_12v = 550;
    private final double moveTo1stBasketPower1_12v = -0.6;
    private final double moveTo1stBasketPower2_12v = 0.6;
    private final double moveto1stBasketDuration2_12v = 750;
    private final double rotateToDrop1stAngle_12v = -45;
    private final double rotateToPickup2ndAngle_12v = -12;
    private final double moveToPickup2ndDuration_12v = 750;
    private final double moveToPickup2ndPower_12v = -0.3;
    private final double moveToDrop2ndDuration_12v = 500;
    private final double rotateToDrop2ndAngle_12v = -45;
    private final double moveToDrop2ndPower_12v = 0.3;
    private final double rotateToPickup3rdAngle_12v = 12;
    private final double moveToPickup3rdDuration_12v = 650;
    private final double moveToPickup3rdPower_12v = -0.3;
    private final double moveToDrop3rdDuration_12v = 500;
    private final double rotateToDrop3rdAngle_12v = -45;

    // 13V Auto Variables
    private final double moveTo1stBasketDuration1_13v = 550;
    private final double moveTo1stBasketPower_13v = -0.6;
    private final double moveTo1stBasketPower2_13v = 0.6;
    private final double moveto1stBasketDuration2_13v = 750;
    private final double rotateToDrop1stAngle_13v = -45;
    private final double rotateToPickup2ndAngle_13v = -12;
    private final double moveToPickup2ndDuration_13v = 750;
    private final double moveToPickup2ndPower_13v = -0.3;
    private final double moveToDrop2ndDuration_13v = 500;
    private final double rotateToDrop2ndAngle_13v = -45;
    private final double moveToDrop2ndPower_13v = 0.3;
    private final double rotateToPickup3rdAngle_13v = 10;
    private final double moveToPickup3rdDuration_13v = 650;
    private final double moveToPickup3rdPower_13v = -0.3;
    private final double moveToDrop3rdDuration_13v = 500;
    private final double rotateToDrop3rdAngle_13v = -45;

    // 14V Auto Variables
    private final double moveTo1stBasketDuration1_14v = 400;
    private final double moveTo1stBasketPower_14v = -0.6;
    private final double moveTo1stBasketPower2_14v = 0.6;
    private final double moveto1stBasketDuration2_14v = 625;
    private final double rotateToDrop1stAngle_14v = -45;
    private final double rotateToPickup2ndAngle_14v = -11;
    private final double moveToPickup2ndDuration_14v = 750;
    private final double moveToPickup2ndPower_14v = -0.3;
    private final double moveToDrop2ndDuration_14v = 500;
    private final double rotateToDrop2ndAngle_14v = -45;
    private final double moveToDrop2ndPower_14v = 0.3;
    private final double rotateToPickup3rdAngle_14v = 12;
    private final double moveToPickup3rdDuration_14v = 650;
    private final double moveToPickup3rdPower_14v = -0.3;
    private final double moveToDrop3rdDuration_14v = 500;
    private final double rotateToDrop3rdAngle_14v = -45;

    //im so proud of this its literally gonna work

    private double batteryVoltage;

    public void updateBatteryVoltage(double voltage) {
        this.batteryVoltage = voltage;
    }

    public int getVariable(String variableName) {
        String section = determineClosestVoltageSection();
        String fullVariableName = variableName + "_" + section;

        try {
            Field field = this.getClass().getDeclaredField(fullVariableName);
            field.setAccessible(true);
            return (int) field.getDouble(this);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            return 0;
        }
    }

    private String determineClosestVoltageSection() {
        if (Math.abs(batteryVoltage - 12.0) <= Math.abs(batteryVoltage - 13.0) &&
                Math.abs(batteryVoltage - 12.0) <= Math.abs(batteryVoltage - 14.0)) {
            return "12v";
        } else if (Math.abs(batteryVoltage - 13.0) <= Math.abs(batteryVoltage - 14.0)) {
            return "13v";
        } else {
            return "14v";
        }
    }
}
