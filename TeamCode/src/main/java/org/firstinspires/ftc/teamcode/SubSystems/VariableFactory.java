package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import java.lang.reflect.Field;
import java.util.HashMap;

public class VariableFactory extends SubsystemBase {

    HashMap<String, Double> lookupTable;

    //im so proud of this its literally gonna work

    private double batteryVoltage;

    public void updateBatteryVoltage(double voltage) {
        this.batteryVoltage = voltage;
    }


    public VariableFactory() {
        lookupTable = new HashMap<>();

        lookupTable.put("moveTo1stBasketDuration1_12v", 550d);
        lookupTable.put("moveTo1stBasketPower1_12v", -0.6d);
        lookupTable.put("moveTo1stBasketPower2_12v", 0.6d);
        lookupTable.put("moveto1stBasketDuration2_12v", 750d);
        lookupTable.put("rotateToDrop1stAngle_12v", -45d);
        lookupTable.put("rotateToPickup2ndAngle_12v", -12d);
        lookupTable.put("moveToPickup2ndDuration_12v", 750d);
        lookupTable.put("moveToPickup2ndPower_12v", -0.3d);
        lookupTable.put("moveToDrop2ndDuration_12v", 500d);
        lookupTable.put("rotateToDrop2ndAngle_12v", -45d);
        lookupTable.put("moveToDrop2ndPower_12v", 0.3d);
        lookupTable.put("rotateToPickup3rdAngle_12v", 12d);
        lookupTable.put("moveToPickup3rdDuration_12v", 650d);
        lookupTable.put("moveToPickup3rdPower_12v", -0.3d);
        lookupTable.put("moveToDrop3rdDuration_12v", 500d);
        lookupTable.put("rotateToDrop3rdAngle_12v", -45d);

        // 13V Auto Variables
        lookupTable.put("moveTo1stBasketDuration1_13v", 550d);
        lookupTable.put("moveTo1stBasketPower_13v", -0.6d);
        lookupTable.put("moveTo1stBasketPower2_13v", 0.6d);
        lookupTable.put("moveto1stBasketDuration2_13v", 750d);
        lookupTable.put("rotateToDrop1stAngle_13v", -45d);
        lookupTable.put("rotateToPickup2ndAngle_13v", -12d);
        lookupTable.put("moveToPickup2ndDuration_13v", 750d);
        lookupTable.put("moveToPickup2ndPower_13v", -0.3d);
        lookupTable.put("moveToDrop2ndDuration_13v", 500d);
        lookupTable.put("rotateToDrop2ndAngle_13v", -45d);
        lookupTable.put("moveToDrop2ndPower_13v", 0.3d);
        lookupTable.put("rotateToPickup3rdAngle_13v", 10d);
        lookupTable.put("moveToPickup3rdDuration_13v", 650d);
        lookupTable.put("moveToPickup3rdPower_13v", -0.3d);
        lookupTable.put("moveToDrop3rdDuration_13v", 500d);
        lookupTable.put("rotateToDrop3rdAngle_13v", -45d);

        // 14V Auto Variables
        lookupTable.put("moveTo1stBasketDuration1_14v", 400d);
        lookupTable.put("moveTo1stBasketPower_14v", -0.6d);
        lookupTable.put("moveTo1stBasketPower2_14v", 0.6d);
        lookupTable.put("moveto1stBasketDuration2_14v", 625d);
        lookupTable.put("rotateToDrop1stAngle_14v", -45d);
        lookupTable.put("rotateToPickup2ndAngle_14v", -11d);
        lookupTable.put("moveToPickup2ndDuration_14v", 750d);
        lookupTable.put("moveToPickup2ndPower_14v", -0.3d);
        lookupTable.put("moveToDrop2ndDuration_14v", 500d);
        lookupTable.put("rotateToDrop2ndAngle_14v", -45d);
        lookupTable.put("moveToDrop2ndPower_14v", 0.3d);
        lookupTable.put("rotateToPickup3rdAngle_14v", 12d);
        lookupTable.put("moveToPickup3rdDuration_14v", 650d);
        lookupTable.put("moveToPickup3rdPower_14v", -0.3d);
        lookupTable.put("moveToDrop3rdDuration_14v", 500d);
        lookupTable.put("rotateToDrop3rdAngle_14v", -45d);
    }

    public double getVariable(String variableName) {
        String section = determineClosestVoltageSection();
        String fullVariableName = variableName + "_" + section;
        if (lookupTable.containsKey(fullVariableName)) {
            return lookupTable.get(fullVariableName);
        }
        return 0;
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
