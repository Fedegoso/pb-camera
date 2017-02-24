#include "cam_reader.h"

static struct cam_position* cam_pos;

int cam_init(){
    int fd;
    /*cam_pos - mmapped array of structs */

    /* Open a file for writing.
     *  - Creating the file if it doesn't exist.
     *  - Truncating it to 0 size if it already exists. (not really needed)
     *
     * Note: "O_WRONLY" mode is not sufficient when mmapping.
     */
    fd = open(FILEPATH, O_RDWR, S_IRUSR);
    if (fd == -1)
    {
        // perror("Error opening file for writing");
        // exit(EXIT_FAILURE);
        return -1;
    }

    /* Now the file is ready to be mmapped.  */
    cam_pos = (struct cam_position *)mmap(NULL, FILESIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (cam_pos == MAP_FAILED)
    {
        close(fd);
        // perror("Error mmapping the file");
        // exit(EXIT_FAILURE);
        return -1;
    }
    memset(cam_pos, '\0', sizeof(struct cam_position));

    /* Retrieve cam pid */
    FILE *pid_process = fopen(PID_FILEPATH, "r");
    if(pid_process==NULL){
        // fprintf(stderr, "Opening file pid_process failed!\n");
        return -1;
    }
    fscanf(pid_process, "%d\n", &pid);
    fflush(pid_process);
    fclose(pid_process);
    return 0;
}

int cam_read(struct cam_position* cam_pos_main){
    /* try lock mutex, if free than lock */
    if(pthread_mutex_trylock(&cam_pos->pmutex)){
         return -1;
    }
    else{
        cam_pos_main->x = cam_pos->x;
        cam_pos_main->y = cam_pos->y;
        /* Unlock mutex */
        if(pthread_mutex_unlock(&cam_pos->pmutex)){
            return -1;
        }
    }
    return 0;
}

/*int cam_start_white(){
    if(kill(pid, START_WHITE_CAM)){
        return -1;
    }
    else{
        return 0;
    }
}

int cam_start_black(){
    if(kill(pid, START_BLACK_CAM)){
        return -1;
    }
    else{
        return 0;
    }
}

int cam_pause(){
    if(kill(pid, PAUSE_CAM)){
        return -1;
    }
    else{
        return 0;
    }
}

int cam_close(){
    if(kill(pid, CLOSE_CAM)){
        return -1;
    }
    else{
        return 0;
    }
}

int cam_is_runing(){
    if(!kill(pid, IS_RUNING)){
        return -1;
    }
    else{
        return 0;
    }
}*/
