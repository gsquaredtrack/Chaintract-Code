import android.util.Log;

import com.allstate.coreEngine.beans.DEMEventInfo;
import com.allstate.coreEngine.beans.DEMTripInfo;
import com.allstate.coreEngine.beans.DEMError;
import com.allstate.coreEngine.constants.DEMEventCaptureMask;
import com.allstate.coreEngine.driving.DEMDrivingEngineManager;

public void registerDriveEngineListener() {
    // set this activity as the listener for all driving events
    DEMDrivingEngineManager.getInstance()
        .setEventListener(this);

    // listen to all driving events available
    DEMDrivingEngineManager.getInstance()
        .registerForEventCapture(DEMEventCaptureMask.DEM_EVENT_CAPTURE_ALL);
}

public class MainActivity extends AppCompatActivity implements DEMDrivingEngineManager.EventListener {

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // set context for the Driving Engine
        DEMDrivingEngineManager.setContext(this.getApplicationContext());

        // register this activity as a listener
        registerDriveEngineListener();

        // start Driving Engine
        DEMDrivingEngineManager.getInstance().startEngine();
    }

    public void registerDriveEngineListener() {
        // set this activity as the listener for all driving events
        DEMDrivingEngineManager.getInstance()
            .setEventListener(this);

        // listen to all driving events available
        DEMDrivingEngineManager.getInstance()
            .registerForEventCapture(DEMEventCaptureMask.DEM_EVENT_CAPTURE_ALL);
    }

    @Override
    public String onTripRecordingStarted() {
        Log.d("DriveEngineSample", "Trip Recording Started");
        return null;
    }

    @Override
    public void onTripRecordingStarted(DEMTripInfo demTripInfo) {
        Log.d("DriveEngineSample", "Trip Recording Started - Initial Draft");
    }

    @Override
    public void onTripInformationSaved(DEMTripInfo demTripInfo, boolean completionFlag) {
        Log.d("DriveEngineSample", "Trip Saved");
    }

    @Override
    public void onTripRecordingStopped() {
        Log.d("DrivingEngineSample", "Trip Recording Stopped");
    }

    @Override
    public void onInvalidTripRecordingStopped() {
        Log.d("DriveEngineSample", "Trip Recording Stopped - Invalid Trip");
    }

    @Override
    public void onBrakingDetected(DEMEventInfo demEventInfo) {
        Log.d("DriveEngineSample", "Braking Detected");
    }

    @Override
    public void onAccelerationDetected(DEMEventInfo demEventInfo) {
        Log.d("DriveEngineSample", "Acceleration Detected");
    }

    @Override
    public void onStartOfSpeedingDetected(DEMEventInfo demEventInfo) {
        Log.d("DriveEngineSample", "Started Speeding Detected");
    }

    @Override
    public void onEndOfSpeedingDetected(DEMEventInfo demEventInfo) {
        Log.d("DriveEngineSample", "Stopped Speeding Detected");
    }

    @Override
    public void onError(DEMError demError) {
        Log.d("DriveEngineSample", "Drive Error Detected");
    }

    @Override
    public String onRequestMetaData() { return null; }

    @Override
    public void onGpsAccuracyChangeDetected(int gpsAccuracyLevel) { }
}