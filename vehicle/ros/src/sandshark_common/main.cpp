#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>

#include <sandshark_common/main.h>

namespace bluefin {
 namespace sandshark {
  bool run;

  static void shutdown_handler( int sig ) {
      ROS_INFO( "\n\nReceived shutdown signal (%d).\n\n", sig );
      run = false;
  }

  int app_main(TaskBase& task, int argc, char *argv[]) {

      run = true;
      struct sigaction act;
      act.sa_handler = &shutdown_handler;
      act.sa_flags = SA_RESTART;
      sigaction(SIGINT, &act, NULL);
      sigaction(SIGTERM, &act, NULL);
      sigaction(SIGABRT, &act, NULL);
      
      ros::init( argc, argv, task.getTaskName(), ros::init_options::NoSigintHandler );

      task.startupInitialization();
      ros::start(); //just in case, creating the nodehandle should do this

      while( ros::ok() && run ) {
          task.run();
          ros::spinOnce();
          task.handleSleep();
      }
      
      task.handleShutdown();
      ros::shutdown();

      return EXIT_SUCCESS;
  }
 }
}
