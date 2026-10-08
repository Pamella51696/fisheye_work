package surround.vehicle;

import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Event-driven steering scenarios with smooth ramps. Yaw derived from bicycle model when moving.
 */
public final class VehicleSignalSimulator implements VehicleSignalProvider {

    public enum Scenario {
        IDLE,
        LEFT_TURN,
        RIGHT_TURN
    }

    private static final double MAX_STEERING_DEG = 20.0;
    private static final double WHEELBASE_M = 2.7;
    private static final double SIM_VELOCITY_MPS = 2.0;

    private final AtomicBoolean running = new AtomicBoolean(false);
    private volatile Scenario scenario = Scenario.IDLE;
    private volatile long scenarioStartMs;
    private volatile double steeringDeg;
    private volatile double yawDeg;
    private Thread tickThread;

    public void start() {
        if (!running.compareAndSet(false, true)) {
            return;
        }
        tickThread = new Thread(this::tickLoop, "vehicle-simulator");
        tickThread.setDaemon(true);
        tickThread.start();
    }

    public void stop() {
        running.set(false);
        if (tickThread != null) {
            tickThread.interrupt();
        }
    }

    public void startScenario(String name) {
        Scenario s;
        try {
            s = Scenario.valueOf(name.toUpperCase(Locale.ROOT));
        } catch (IllegalArgumentException e) {
            s = Scenario.RIGHT_TURN;
        }
        if (s == Scenario.IDLE) {
            scenario = Scenario.IDLE;
            steeringDeg = 0.0;
            return;
        }
        scenario = s;
        scenarioStartMs = System.currentTimeMillis();
    }

    @Override
    public VehicleSignal getCurrentSignal() {
        return new VehicleSignal(System.currentTimeMillis(), VehicleSignal.Status.OK,
                steeringDeg, yawDeg, scenario == Scenario.IDLE ? 0.0 : SIM_VELOCITY_MPS);
    }

    private void tickLoop() {
        double lastYaw = 0.0;
        long lastTick = System.currentTimeMillis();
        while (running.get()) {
            long now = System.currentTimeMillis();
            double dt = (now - lastTick) / 1000.0;
            lastTick = now;
            steeringDeg = scenarioSteering(now);
            double yawRate = 0.0;
            if (scenario != Scenario.IDLE && SIM_VELOCITY_MPS > 0.01) {
                double steerRad = Math.toRadians(steeringDeg);
                yawRate = SIM_VELOCITY_MPS / WHEELBASE_M * Math.tan(steerRad);
            }
            lastYaw += Math.toDegrees(yawRate) * dt;
            yawDeg = lastYaw;
            if (scenario != Scenario.IDLE && now - scenarioStartMs > 4000) {
                scenario = Scenario.IDLE;
                steeringDeg = 0.0;
            }
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    private double scenarioSteering(long nowMs) {
        if (scenario == Scenario.IDLE) {
            return 0.0;
        }
        double t = (nowMs - scenarioStartMs) / 1000.0;
        double sign = scenario == Scenario.LEFT_TURN ? -1.0 : 1.0;
        if (t < 2.0) {
            return sign * MAX_STEERING_DEG * (t / 2.0);
        }
        if (t < 4.0) {
            return sign * MAX_STEERING_DEG * (1.0 - (t - 2.0) / 2.0);
        }
        return 0.0;
    }
}
