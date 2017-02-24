#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

#define FILEPATH "/dev/shm/cam_mmaped.bin"
#define PID_FILEPATH "/var/run/cam.pid"
#define FILESIZE sizeof(struct cam_position)

#ifndef STRUCT_CAM_POSITION
#define STRUCT_CAM_POSITION
struct cam_position
{
    int x;
    int y;
    pthread_mutex_t  pmutex;
};
#endif

/* Globals */
int pid = 0;

/* Sginals */
#define START_WHITE_CAM     SIGUSR1
#define PAUSE_CAM           SIGINT
#define START_BLACK_CAM     SIGUSR2
#define CLOSE_CAM           SIGTERM
#define IS_RUNING           0

/* Functions */
int cam_init();
int cam_read(struct cam_position* cam_pos);
int cam_start_white();
#define cam_start_white() kill(pid, START_WHITE_CAM)
int cam_start_black();
#define cam_start_black() kill(pid, START_BLACK_CAM)
int cam_pause();
#define cam_pause() kill(pid, PAUSE_CAM)
int cam_close();
#define cam_close() kill(pid, CLOSE_CAM)
int cam_is_runing();
#define cam_is_runing() (!kill(pid, IS_RUNING))

