package com.bluefinrobotics.bluefingps;

import android.app.Service;
import android.location.Location;
import android.location.LocationManager;
import android.location.LocationListener;
import android.location.LocationProvider;
import android.location.GpsStatus;
import android.os.IBinder;
import android.os.PowerManager;
import android.os.PowerManager.WakeLock;
import android.net.wifi.WifiManager;
import android.net.wifi.WifiManager.WifiLock;
import android.util.Log;
import android.content.Intent;
import android.content.Context;
import android.os.Bundle;
import android.provider.Settings;
import java.util.List;
import android.os.SystemClock;
import android.net.Uri;
import android.app.PendingIntent;
import android.content.Intent;
import android.location.Criteria;
import android.database.sqlite.SQLiteDatabase;
import android.app.AlarmManager;
import android.os.SystemClock;
import java.util.Date;
import java.util.Locale;
import java.util.Calendar;
import java.util.GregorianCalendar;
import java.util.TimeZone;
import java.text.DateFormat;
import java.text.SimpleDateFormat;

public class BluefinAdaptGpsService extends Service {
    private static final String TAG = "BluefinAdaptGPSService";
    private static final boolean DEBUG=true; // Set to false to reduce logcat spam to only errors

    public static final String DATABASE_NAME = "BLUEFINGPSDB";
    public static final String POINTS_TABLE_NAME = "LOCATION_POINTS";
    private final DateFormat timestampFormat = new SimpleDateFormat("yyyyMMddHHmmss");
    private LocationManager lm;
    private LocationListener locationListener;
    private SQLiteDatabase db;

    private static WakeLock mWakeLock;
    private static WifiLock mWifiLock;
    private PowerManager pm;
    private WifiManager wifiManager;

    //private PendingIntent pendIntent;
    public Intent myIntent;

    private static long minTimeMillis = 2000;
    private static long minDistanceMeters = 0;
    private static float minAccuracyMeters = 35;

    private int lastStatus = 0;

    private void startLoggerService() {
        // ---use the LocationManager class to obtain GPS locations---
        lm = (LocationManager) getSystemService(Context.LOCATION_SERVICE);

        locationListener = new MyLocationListener();

        Criteria criteria = new Criteria();
        criteria.setAccuracy(Criteria.ACCURACY_FINE);
        criteria.setCostAllowed(false);
        Settings.Secure.setLocationProviderEnabled(getContentResolver(), LocationManager.PASSIVE_PROVIDER, true);
        lm.requestLocationUpdates(     minTimeMillis, 
                                       minDistanceMeters,
                                       criteria,
                                       locationListener,
                                       null );
        pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
        mWakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "adaptBluefinGPS");
        mWakeLock.setReferenceCounted(false);

        wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);
        mWifiLock = wifiManager.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "adaptBluefinGPS");
        mWifiLock.setReferenceCounted(false);

        if (!mWakeLock.isHeld()) {
            mWakeLock.acquire();
        }
        if (!mWifiLock.isHeld()) {
           mWifiLock.acquire();
        }
    }

    private void shutdownLoggerService() {
        lm.removeUpdates(locationListener);
        if (mWakeLock.isHeld()) {
            mWakeLock.release();
        }
        if (mWifiLock.isHeld()) {
           mWifiLock.release();
        }
    }

    private void initDatabase() {
        db = this.openOrCreateDatabase(DATABASE_NAME, SQLiteDatabase.OPEN_READWRITE, null);
        
        db.execSQL("DROP TABLE IF EXISTS " + POINTS_TABLE_NAME + ";" );        
        db.execSQL("CREATE TABLE IF NOT EXISTS " +
                            POINTS_TABLE_NAME + " (ID INTEGER PRIMARY KEY, GMTTIMESTAMP VARCHAR, EPOCHTIME REAL, LATITUDE REAL, LONGITUDE REAL," +
                                                "ALTITUDE REAL, ACCURACY REAL, SATELLITES INT, SPEED REAL, BEARING REAL);");
        StringBuffer queryBuf = new StringBuffer();
        queryBuf.append("INSERT INTO "+POINTS_TABLE_NAME+
                                " (ID, GMTTIMESTAMP, EPOCHTIME, LATITUDE,LONGITUDE,ALTITUDE,ACCURACY,SATELLITES,SPEED,BEARING) VALUES (" +
                                "1,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL);" );
        db.execSQL( queryBuf.toString() );
        db.close();
        Log.i(TAG, "Database opened ok");
    }

    public class MyLocationListener implements LocationListener { //, GpsStatus.Listener {

/*        public void onGpsStatusChanged( int event ) {

        }*/

        public void onLocationChanged(Location loc) {
            //Log.i( TAG, "got loc!" );
            if (loc != null) {
                Log.i(TAG, loc.getLatitude() + "," +
                             loc.getLongitude() + "," +
                             (loc.hasAltitude() ? loc.getAltitude() : "?" ) + "," +
                             (loc.hasAccuracy() ? loc.getAccuracy() : "?" ) + "," +
                             (loc.hasSpeed() ? loc.getSpeed() : "?" ) + "," +
                             (loc.hasBearing() ? loc.getBearing() : "?" ) + "," + 
                             (loc.hasAccuracy() ? loc.getAccuracy() : "?" ) );

                long time = loc.getTime();
                long currentTime = System.currentTimeMillis(); 

                if( Math.abs( time - currentTime ) > 1000L ) {
                    AlarmManager amgr = (AlarmManager)getSystemService(Context.ALARM_SERVICE);
                    Log.i( TAG, "AMgr is " + amgr.toString() + " Time = " + time + " Cur= " + currentTime + " Diff = " + Math.abs( time - currentTime )	 );
                    amgr.setTime( time );
                }
                try {
                    if (loc.hasAccuracy() && loc.getAccuracy() <= minAccuracyMeters) {
                        GregorianCalendar greg = new GregorianCalendar();
                        TimeZone tz = greg.getTimeZone();
                        int offset = tz.getOffset(System.currentTimeMillis());
                        greg.add(Calendar.SECOND, (offset/1000) * -1);
                        StringBuffer queryBuf = new StringBuffer();
                        Log.i( TAG, "Number of satellites = " + loc.getExtras().getInt( "satellites" ) );
                        queryBuf.append("UPDATE "+POINTS_TABLE_NAME+ " SET " +
                                "GMTTIMESTAMP = " + "'"+timestampFormat.format(greg.getTime())+"',"+
                                "EPOCHTIME = " + currentTime + "," +
                                "LATITUDE = " + loc.getLatitude() +"," +
                                "LONGITUDE = " + loc.getLongitude()+"," +
                                "ALTITUDE = " + (loc.hasAltitude() ? loc.getAltitude() : "NULL") +"," +
                                "ACCURACY = " + (loc.hasAccuracy() ? loc.getAccuracy() : "NULL") +"," +
                                "SATELLITES = " + loc.getExtras().getInt( "satellites" ) + "," +
                                "SPEED = " + (loc.hasSpeed() ? loc.getSpeed() : "NULL") +"," +
                                "BEARING = " + (loc.hasBearing() ? loc.getBearing() : "NULL") + 
                                " WHERE ID = 1;" );
                                
                        Log.i( TAG, queryBuf.toString() );
                        db = openOrCreateDatabase(DATABASE_NAME, SQLiteDatabase.OPEN_READWRITE, null);
                        db.execSQL(queryBuf.toString());
                    } 
                } catch (Exception e) {
                    Log.e( TAG, e.toString() );
                } finally {
                    if( db.isOpen() ) {
                        db.close();
                    }
                }
            }
        }

        public void onProviderDisabled(String provider) {
            if( DEBUG ) { 
                Log.i(TAG, "onProviderDisabled: " + provider );
            }
        }

        public void onProviderEnabled(String provider) {
            if( DEBUG ) {
                Log.i(TAG, "onProviderEnabled: " + provider );
            }
        }

        public void onStatusChanged(String provider, int status, Bundle extras) {
            String showStatus = null;
            if( status == LocationProvider.AVAILABLE ) {
                showStatus = "Available";
            }
            if( status == LocationProvider.TEMPORARILY_UNAVAILABLE ) {
                showStatus = "Temporarily Unavailable";
            }
            if( status == LocationProvider.OUT_OF_SERVICE ) {
                showStatus = "Out of Service";
            }
            if( status != lastStatus && DEBUG) {
                Log.i( TAG, "new status: " + showStatus );
            }
            lastStatus = status;
        }

    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startid) { 
        startLoggerService();
        toggleGps( true, false );
        Log.i( TAG, "BluefinGPS Start - Intent was from " + intent.toString() );

        return START_STICKY;
    }

    @Override
    public IBinder onBind(Intent intent) {
        if(DEBUG) {
            Log.i(TAG, "onBind ");
        }
        return null;
    }
    
    @Override
    public void onCreate() {
        super.onCreate();
        myIntent = new Intent("com.bluefinrobotics.bluefingps.BluefinAdaptGpsService" );
        initDatabase();
        Log.i( TAG, "BluefinGPS Created" );
    }
    
    @Override
    public void onDestroy() {
        super.onDestroy();
        shutdownLoggerService();
        Log.i( TAG, "BluefinGPS Destroyed" );
        toggleGps( false, false );
    }
    
    private long gpsStartmsSinceBoot;
    private boolean toggledGps;
    private boolean checkIsGpsActive=false;

    private void toggleGps( boolean enable, boolean ignoreProvider ){
        try {
            String provider = Settings.Secure.getString(getContentResolver(), 
                                                        Settings.Secure.LOCATION_PROVIDERS_ALLOWED);
            if( DEBUG ) {
                Log.d( TAG, "toggleGPS() allowed providers are: ("+provider+")");
            }

            if( ( provider.contains( LocationManager.GPS_PROVIDER ) == enable ) && !ignoreProvider ) {
                Log.i( TAG, "toggleGps(): Location provider allowed includes "+LocationManager.GPS_PROVIDER+"="+
                            String.valueOf(provider.contains(LocationManager.GPS_PROVIDER))+
                            ", so not attempting to set GPS receiver to an enabled state of: "+String.valueOf(enable));
                return; // the GPS is already in the requested enable state
            } else if( ignoreProvider ){
                Log.w( TAG, "toggleGps() setting gps toggle to: " +
                            String.valueOf(provider.contains(LocationManager.GPS_PROVIDER))+
                            " and ignoring mismatch between expected and actual providers alowed!");
                toggledGps=true;
            }
                     
                
            // Toggle all known providers
            List<String> matchingProviders = lm.getAllProviders();
            for (String aProvider: matchingProviders) {
                Settings.Secure.setLocationProviderEnabled(getContentResolver(), aProvider, enable);
            }
            
            //Settings.Secure.setLocationProviderEnabled(getContentResolver(), LocationManager.GPS_PROVIDER, enable);
            //Settings.Secure.setLocationProviderEnabled(getContentResolver(), "hybrid", enable);
            provider = Settings.Secure.getString(getContentResolver(), 
                                       Settings.Secure.LOCATION_PROVIDERS_ALLOWED);
            if( DEBUG ) { 
                Log.d( TAG, "toggleGPS(); After the toggle the allowed providers are: ("+provider+")" );
            }
            
            //if(DEBUG) Log.i(TAG, "toggleGps(): gps, hybrid, and network location providers set to "+String.valueOf(enable));
            String state;
            if(enable) {
                state="on";
                gpsStartmsSinceBoot=SystemClock.uptimeMillis();//timer record of the current time in ms
            } else {
                state="off";
            }
            Log.i( TAG, System.currentTimeMillis()+", "+state);
            checkIsGpsActive=!checkIsGpsActive; // toggle active setting
            // work around, exploiting a security hole to turn on/off gps by poking the setting
            final Intent poke = new Intent();
            poke.setClassName("com.android.settings", "com.android.settings.widget.SettingsAppWidgetProvider"); 
            poke.addCategory(Intent.CATEGORY_ALTERNATIVE);
            poke.setData(Uri.parse("3"));  
            //SET 3 BUTTON_GPS = 3; as defined in packages/apps/Settings/src/com/android/settings/widget/SettingsAppWidgetProvider.java
            sendBroadcast(poke);
            // I don't have any way to get the current gps setting only record if the providers are enabled so make sure always set in lock step
                 
            if( DEBUG ) {
                Log.d(TAG, "toggleGPS(): Toggling gps on/off and location providers to "+String.valueOf(enable));
            }

        } catch(Exception e) {
            if(DEBUG) {
                Log.d(TAG, "toggleGPS(): Location exception thrown in toggling GPS setting "+e);
            }
        }
    }
}

