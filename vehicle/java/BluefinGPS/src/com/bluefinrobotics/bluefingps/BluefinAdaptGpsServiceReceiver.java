package com.bluefinrobotics.bluefingps;

import android.app.AlarmManager;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

import com.bluefinrobotics.bluefingps.BluefinAdaptGpsService;

public class BluefinAdaptGpsServiceReceiver extends BroadcastReceiver {
    private static final String TAG = "BluefinAdaptGpsServiceReceiver";
    private static boolean DEBUG = true;

    @Override
    public void onReceive(Context context, Intent intent) {
        Log.i( TAG, "Got Receiver intent -- " + intent.toString() );
        //if( intent.getAction().equals(Intent.ACTION_BOOT_COMPLETED) ) {
            //this is starting the service from a BroadcastReceiver of BOOT_COMPLETED
        if( !intent.getAction().equals("com.bluefinrobotics.bluefingps.ADAPT_STOP_GPS") ) {            
            if(DEBUG) {
                Log.i(TAG, "onReceive - Started from intent" + intent.toString() );
            }
            Intent myIntent = new Intent("com.bluefinrobotics.bluefingps.BluefinAdaptGpsService");
            context.startService(myIntent);
        } else {
            if(DEBUG) {
                Log.i(TAG, "onReceive - Stopped from ADAPT_STOP_GPS intent");
            }
            Intent myIntent = new Intent("com.bluefinrobotics.blufingps.BluefinAdaptGpsService");
            PendingIntent pilocal =PendingIntent.getService(context, 0, myIntent, 0);
            AlarmManager alarmManager = (AlarmManager) context.getSystemService(Context.ALARM_SERVICE);
            alarmManager.cancel(pilocal);
            pilocal.cancel();
            context.stopService(myIntent);
        }
    }
}
