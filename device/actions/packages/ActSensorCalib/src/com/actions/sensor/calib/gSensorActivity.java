package com.actions.sensor.calib;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.lang.Process;
import java.lang.Runtime;


public class gSensorActivity extends Activity implements SensorEventListener {

	private final String TAG = "gSensorActivity";
	private final int mButtonDelay = 1000;
	
	private Handler mHandler;	
	private SensorManager sm = null;
	private SensorControl sc = null;
	private boolean mCalibMode = false;

	private TextView mTextView;
	private SensorHost mSensorHost;
	private Button mButtonCalib;
	private Button mButtonReset;
	

    private Runnable mCalibRunnable = new Runnable() {
        
		public void run() {
			mButtonCalib.setClickable(true);
			mCalibMode = false;
            
			if(sc.calibration_type == sc.CALI_TYPE_INPUT)
			{
				Log.i(TAG, "calibration_type: CALI_TYPE_INPUT");
			// calibrate g-sensor
			sc.runCalib();

			// get new calib data
			String calib = sc.getCalibValue();
			Log.i(TAG, "Calib: " + calib);

			// write calibration file
			String file = calib + "\n";
			sc.writeCalibFile(file);
			}
			else if(sc.calibration_type == sc.CALI_TYPE_IIO)
			{
				Log.i(TAG, "calibration_type: CALI_TYPE_IIO");
			Log.i(TAG, "start inv self_test...");
			
			String args = "inv_self_test-shared -w";
            try
            {
                Process process = Runtime.getRuntime().exec(args);
				process.waitFor();
				
				//get the err line
                InputStream stderr = process.getErrorStream();
                InputStreamReader isrerr = new InputStreamReader(stderr);
                BufferedReader brerr = new BufferedReader(isrerr);

				//get the output line  
                InputStream outs = process.getInputStream();
                InputStreamReader isrout = new InputStreamReader(outs);
                BufferedReader brout = new BufferedReader(isrout);

                String errline = null;
                String result = "";

				// get the whole error message string  
                while ( (errline = brerr.readLine()) != null)
                {
                    result += errline;
                    result += "\n";
                } 

                if( result != "" )
                {
                    // put the result string on the screen
                	Log.i(TAG, result);

                }
                
                String outline = null;
                String out_result = "";
                 // get the whole standard output string
                 while ( (outline = brout.readLine()) != null)
                {
                	 out_result += outline;
                	 out_result += "\n";
                }
                if( out_result != "" )
                {
                    // put the result string on the screen
                	Log.i(TAG, out_result);

                }
               
				

             }catch(Throwable t)
             {
                 t.printStackTrace();
             }
			}

			// show info
			Toast.makeText(gSensorActivity.this, R.string.info_calib, 
					Toast.LENGTH_LONG).show();						
		}
	};
    
	private View.OnClickListener mBtnCalibListener = new View.OnClickListener() {

		public void onClick(View v) {
			mButtonCalib.setClickable(false);
			mCalibMode = true;
            
			if(sc.calibration_type == sc.CALI_TYPE_INPUT)
			{
			// reset calibration
			sc.resetCalib();
			}

			mHandler.postDelayed(mCalibRunnable, mButtonDelay);
		}
	};

    private Runnable mResetRunnable = new Runnable() {
        
		public void run() {
			mButtonReset.setClickable(true);
			mCalibMode = false;

			if(sc.calibration_type == sc.CALI_TYPE_INPUT)
			{
			// get new calib data
			String calib = sc.getCalibValue();
			Log.i(TAG, "Calib: " + calib);

			// write calibration file
			String file = calib + "\n";
			sc.writeCalibFile(file);
			}
			else if(sc.calibration_type == sc.CALI_TYPE_IIO)
			{
			
            Log.i(TAG, " clear the MLCAL_FILE...");
            String args = "inv_self_test-shared -c";
            try
            {
                Process process = Runtime.getRuntime().exec(args);
				process.waitFor();
				
				//get the err line
                InputStream stderr = process.getErrorStream();
                InputStreamReader isrerr = new InputStreamReader(stderr);
                BufferedReader brerr = new BufferedReader(isrerr);

				//get the output line  
                InputStream outs = process.getInputStream();
                InputStreamReader isrout = new InputStreamReader(outs);
                BufferedReader brout = new BufferedReader(isrout);

                String errline = null;
                String result = "";

				// get the whole error message string  
                while ( (errline = brerr.readLine()) != null)
                {
                    result += errline;
                    result += "/n";
                } 

                if( result != "" )
                {
                    // put the result string on the screen
                	Log.i(TAG, result);

                }
                
                String outline = null;
                String out_result = "";
                 // get the whole standard output string
                 while ( (outline = brout.readLine()) != null)
                {
                	 out_result += outline;
                	 out_result += "/n";
                }
                if( out_result != "" )
                {
                    // put the result string on the screen
                	Log.i(TAG, out_result);

                }
             }catch(Throwable t)
             {
                 t.printStackTrace();
             }
			}
			

			// show info
			Toast.makeText(gSensorActivity.this, R.string.info_reset, 
					Toast.LENGTH_LONG).show();
		}
	};
    
	private View.OnClickListener mBtnResetListener = new View.OnClickListener() {

		public void onClick(View v) {
			mButtonReset.setClickable(false);
			mCalibMode = true;

			if(sc.calibration_type == sc.CALI_TYPE_INPUT)
			{
			// reset calibration
			sc.resetCalib();
			}

			mHandler.postDelayed(mResetRunnable, mButtonDelay);
		}
	};
	
	private void findViews() {
		mTextView = (TextView) findViewById(R.id.mTextView);
		mButtonCalib = (Button) findViewById(R.id.buttonCalib);
		mButtonReset = (Button) findViewById(R.id.buttonReset);
		mSensorHost = (SensorHost) findViewById(R.id.mSensorHost);
	}

	private void setListensers() {
		mButtonCalib.setOnClickListener(mBtnCalibListener);
		mButtonReset.setOnClickListener(mBtnResetListener);		
	}

	public void onCreate(Bundle paramBundle) {
		super.onCreate(paramBundle);
		setContentView(R.layout.gsensor);
		findViews();
		Log.i(TAG, "onCreate...");
		mHandler = new Handler();
		sm = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		sc = new SensorControl(this);
		
		setListensers();
	}

	protected void onDestroy() {
		super.onDestroy();
		mHandler = null;
		sm = null;
		sc = null;
	}

	protected void onPause() {
		super.onPause();
		sm.unregisterListener(this);
        mHandler.removeCallbacks(mCalibRunnable);
        mHandler.removeCallbacks(mResetRunnable);
	}

	protected void onResume() {
		super.onResume();
		sm.registerListener(this,
				sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
				SensorManager.SENSOR_DELAY_NORMAL);
	}

	public void onAccuracyChanged(Sensor paramSensor, int paramInt) {
	}

	public void onSensorChanged(SensorEvent e) {
		if ((e != null) && (e.values.length == 3)) {
			//filter event in calib mode
			if(mCalibMode)
				return;

			if (mTextView != null) {
				String txt = String.format("X: %.3f, Y: %.3f, Z: %.3f",
						e.values[0], e.values[1], e.values[2]);
				mTextView.setText(txt);
				mTextView.setTextColor(-16711681);
			}
			if (mSensorHost != null)
				mSensorHost.onSensorChanged(e);
		}
	}
}
