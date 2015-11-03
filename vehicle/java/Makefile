build: 
	cd BluefinGPS && JAVA_HOME=~/ADAPT/jdk1.6.0_45 ANDROID_HOME=~/ADAPT/android-sdk-linux ant release        

install: build 
	JAVA_HOME=~/ADAPT/jdk1.6.0_45 java -jar BluefinGPS/signapk.jar BluefinGPS/platform.x509.pem BluefinGPS/platform.pk8 BluefinGPS/bin/BluefinGPS-release-unsigned.apk BluefinGPS/bin/BluefinGPS-release-signed.apk 
clean: 
	@rm -rf BluefinGPS/bin
	@rm -rf BluefinGPS/gen

deploy: install
	adb uninstall com.bluefinrobotics.bluefingps
	adb install BluefinGPS/bin/BluefinGPS-release-signed.apk

deploy-ssh: install
	scp BluefinGPS/bin/BluefinGPS-release-signed.apk root@$(SSHTARGET):/data/app
	ssh root@$(SSHTARGET) 'setprop service.adb.tcp.port 5555;\
                            stop adbd;\
                            start adbd;\
                            adb devices;\
                            adb connect localhost;\
                            echo uninstalling...;\
                            adb uninstall com.bluefinrobotics.bluefingps;\
                            echo installing...;\
                            adb install /data/app/BluefinGPS-release-signed.apk'
               
.PHONY: build install clean
                

