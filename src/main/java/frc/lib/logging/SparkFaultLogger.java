package frc.lib.logging;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.CanIDs;

/**
 * Custom logging class for Spark Max/Flex devices to log faults
 * and warnings into a nice readable string.
 */
@CustomLoggerFor(SparkBase.class)
public class SparkFaultLogger extends ClassSpecificLogger<SparkBase> {

    private Alert[] motorFaults;
    private Alert[] motorWarnings;

  public SparkFaultLogger() {
    super(SparkBase.class);
    

    motorFaults = new Alert[64];
    motorWarnings = new Alert[64];

    // Create for all 64 possible CAN ids
    for(int j=0; j < 64; j++)
    {
        motorFaults[j] = new Alert("null", AlertType.kError);
        motorWarnings[j] = new Alert("null", AlertType.kWarning);
    }
  }

  @Override
  public void update(EpilogueBackend backend, SparkBase motor) {

    Faults f = motor.getFaults();
    String s = "";

    int id = motor.getDeviceId();

    if(f.can)
        s += "CAN, ";
    if(f.escEeprom)
        s += "escEeprom, ";
    if(f.firmware)
        s += "firmware, ";
    if(f.gateDriver)
        s += "gateDriver, ";
    if(f.motorType)
        s += "motorType, ";
    if(f.sensor)
        s += "sensor, ";
    if(f.temperature)
        s += "temperature, ";
    if(f.other)
        s += "other, ";

    backend.log("Faults", s);

    if(s.length() > 0)
    {
        motorFaults[id].setText(getMotorName(id)+" faults: "+s);
        motorFaults[id].set(true);
    }
    else
        motorFaults[id].set(false);


    Warnings w = motor.getWarnings();
    s = "";
    if(w.brownout)
        s += "brownout, ";
    if(w.escEeprom)
        s += "escEeprom, ";
    if(w.extEeprom)
        s += "extEeprom, ";
    if(w.hasReset)
        s += "hasReset, ";
    if(w.other)
        s += "other, ";
    if(w.overcurrent)
        s += "overcurrent, ";
    if(w.sensor)
        s += "sensor, ";
    if(w.stall)
        s += "stall, ";

    backend.log("Warnings", s);

    if(s.length() > 0)
    {
        motorWarnings[id].setText(getMotorName(id)+" warnings: "+s);
        motorWarnings[id].set(true);
    }
    else
        motorWarnings[id].set(false);
  }

  public static String getMotorName(int id)
  {
    return CanIDs.URCL_IDS.get(id) + " (id "+id+")";
  }
}
