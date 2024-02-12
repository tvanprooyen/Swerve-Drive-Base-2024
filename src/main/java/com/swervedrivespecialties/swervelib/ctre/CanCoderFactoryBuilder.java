package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

import edu.wpi.first.wpilibj.Timer;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public CanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANcoderConfiguration config = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                    .withSensorDirection(direction == Direction.CLOCKWISE ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive)
                    .withMagnetOffset(configuration.getOffset() / (2 * Math.PI)));

            CANcoder encoder = new CANcoder(configuration.getId(), configuration.getCanbus());

            CtreUtils.checkCtreError(encoder.getConfigurator().apply(config, 0.25), "Failed to configure CANCoder");

            encoder.optimizeBusUtilization();

            CtreUtils.checkCtreError(encoder.getAbsolutePosition().setUpdateFrequency(1000.0 / periodMilliseconds, 0.25), "Failed to configure CANCoder update rate");
            CtreUtils.checkCtreError(encoder.getMagnetHealth().setUpdateFrequency(1000.0 / periodMilliseconds, 0.25), "Failed to configure CANCoder Magnet Helth update rate");

            StatusSignal<MagnetHealthValue> magHealth = encoder.getMagnetHealth().refresh();

            if(magHealth.getValue() != MagnetHealthValue.Magnet_Green) {
                CtreUtils.checkCtreError(magHealth.getStatus(), encoder.getDeviceID()+" Magnet Health is " + magHealth.getValue() +". Check if CANCoder Magnet is secure and undamaged.");                
            }

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private int ATTEMPTS = 0;

        private final CANcoder encoder;

        private double CANcoderInitTime = 0.0;

        private EncoderImplementation(CANcoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            StatusSignal<Double> angleCode = encoder.getAbsolutePosition();

            double angle = 0;

            //Using method bellow
            /* for (int i = 0; i < ATTEMPTS; i++) {
                if (angleCode.getStatus() == StatusCode.OK) break;
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) { }
                angleCode = encoder.getAbsolutePosition();
            } */


            //Wait until we get a good signal from the CANCoder, this process can take up to 1 second
            //We mostly need to wait for the encoder to power up initally
            for (int i = 0; i < 100; i++) {
                //Grab Angle
                angleCode = encoder.getAbsolutePosition().refresh();

                //If the sensor is good, then update the user and continue
                if(angleCode.getStatus() == StatusCode.OK) {

                    //Oops, Took longer than we thought!
                    if(CANcoderInitTime > 0.03) {
                        CtreUtils.reportWarning("CANCoder "+encoder.getDeviceID()+" absolute position grabbed @ " + angleCode.getValue() + " After: " + CANcoderInitTime + "s and " + ATTEMPTS + " tries");
                    }

                    //If our sensor is good, then use angle and scale 0-1 to radians
                    angle = angleCode.getValue() * 2.0 * Math.PI;
                    angle %= 2.0 * Math.PI;
                    if (angle < 0.0) {
                        angle += 2.0 * Math.PI;
                    }
                    break;
                }

                //Mark down for reporting
                CANcoderInitTime += 0.01;
                ATTEMPTS += 1;
                
                //Wait a little for the sensor
                Timer.delay(0.0100);
            }

            StatusSignal<MagnetHealthValue> magHealth = encoder.getMagnetHealth().refresh();

            //Check if magnet health is good, otherwise return no angle adjustment
            if(magHealth.getValue() == MagnetHealthValue.Magnet_Orange) {
                CtreUtils.checkCtreError(magHealth.getStatus(), encoder.getDeviceID()+" Magnet is inaccurate , check Magnet!");
            } else if(magHealth.getValue() != MagnetHealthValue.Magnet_Green) {
                CtreUtils.checkCtreError(magHealth.getStatus(), encoder.getDeviceID()+" Magnet is unreliable, defaulting to motor encoder.");
                return 0;
            } 

            //Had a hard time getting the posision of the encoder!
            CtreUtils.checkCtreError(angleCode.getStatus(), "Failed to retrieve CANcoder "+encoder.getDeviceID()+" absolute position after "+ATTEMPTS+" tries");

            return angle;
        }

        @Override
        public Object getInternal() {
            return this.encoder;
        }

        @Override
        public boolean isEncoderOK() {
            return encoder.getAbsolutePosition().refresh().getStatus() == StatusCode.OK && (encoder.getMagnetHealth().refresh().getValue() == MagnetHealthValue.Magnet_Green || encoder.getMagnetHealth().refresh().getValue() == MagnetHealthValue.Magnet_Orange);
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
