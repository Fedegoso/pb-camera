// #include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>		// Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <signal.h>
#include <sys/types.h>
#include <syslog.h>

#define angulo 0.84
#define desloc 139
#define siga 0
#define direita 1
#define esquerda -1
#define sensitivity_branco 200
#define sensitivity_preto 20
#define camera_entrada 0

#define FILEPATH "/dev/shm/cam_mmaped.bin"
#define FILESIZE (sizeof(struct Axis_t))

#define PID_FILEPATH "/var/run/cam.pid"

using namespace cv;
using namespace std;

#include <sys/time.h>
#include <sys/resource.h>

double get_time()
{
	struct timeval t;
	struct timezone tzp;
	gettimeofday(&t, &tzp);
	return t.tv_sec + t.tv_usec*1e-6;
}

struct camera_t {
	// pthread_mutex_t mutex;
	int x, y, d;
	bool runing, obst;
	Mat *frame_branco, *frame_preto;
	Mat *gray, *canal_verde, *threshold_img, *threshold_img_verde;
	Mat *channel;
	int i, ii, maior_areaB, maior_areaP, pos_maior_areaB, pos_maior_areaP;
	int atual_areaB, atual_areaP, contoursB_size, contoursP_size;
	double time;
	vector<vector<Point> >*contoursB; 
	vector<vector<Point> >*contoursP;
	vector<Vec4i> *hierarchy;
	VideoCapture *cap;
};

struct Axis_t *axis;  /* mmapped array of structs */

struct Axis_t{
    int x;
    int y;
    pthread_mutex_t  pmutex;
};

bool stop = true, sig_user1 = false, 
sig_user2 = false, sig_quit = true, sig_term = true;

void stopper(int signo){
	switch (signo){
		case SIGINT: /* process by its controlling terminal when a user wishes to interrupt the process. 
		This is typically initiated by pressing Ctrl+C*/
			stop = false;
			sig_user1 = false;
			sig_user2 = false;
		break;
		
		case SIGUSR1:
			sig_user1 = true;
		break;

		case SIGUSR2:
			sig_user2 = true;
		break;
		
		case SIGQUIT:
			stop = false;
			sig_quit = false;
		break;

		case SIGTERM:
			stop = false;
			sig_term = false;
		break;
		// case SIGSTOP : /* operating system to stop a process for later resumption*/
		// 	sig_stop = false;
		// break;

		// case SIGCONT : /* operating system to continue (restart) a process previously paused by the SIGSTOP*/
		// 	sig_cont = false;
		// break;
		
		// case SIGBUS :  incorrect memory access alignment or non-existent physical address
		// 	sig_bus = false;
		// break;
 
		// case SIGFPE : /* floating-point exception or an erroneous arithmetic operation, such as division by zero*/
		// 	sig_fpe = false;
		// break;

		// case SIGSEGV : /* segmentation violation*/
		// 	sig_segv = false;
		// break;

		default: 
			//
		break;
	}
}

typedef struct camera_t* camera; /* camera é um ponteiro para a estrutura camera_t */

camera cameraCreate() {
	camera cam = (camera)malloc(sizeof(struct camera_t));
	cam->frame_branco = new Mat;
	cam->frame_preto = new Mat;
	cam->gray = new Mat;
	cam->canal_verde = new Mat;
	cam->threshold_img = new Mat;
	cam->threshold_img_verde = new Mat;
	cam->runing=true;
	cam->obst=true;
	cam->cap = new VideoCapture(camera_entrada);
	cam->contoursB = new vector<vector<Point> >;
	cam->contoursP = new vector<vector<Point> >;
	cam->hierarchy = new vector<Vec4i>;
	cam->channel = new Mat[3];
	cam->pos_maior_areaB=-1;
	cam->pos_maior_areaP=-1;
	return cam;
}

void* whiteTrakkingThread(void *arg) {
	camera cam = (camera)arg;
	int cont = 0, ret=0;
    // char str[] = "./Medidas_reta/threshold_img%d.jpg";
    // float func;
    int centro_x, centro_y, x1, x2, time_fps=0, cont_fps=0;
    float time_frame=0;
    double geter=0, seter=0;

    //  double fps = video.get(CV_CAP_PROP_FPS);
    // CV_CAP_PROP_CONTRAST 
    // CV_CAP_PROP_BRIGHTNESS
    // CV_CAP_PROP_SATURATION
    // CV_CAP_PROP_EXPOSURE
    // CV_CAP_PROP_GAIN
	// geter = cam->cap->get(CV_CAP_PROP_SATURATION);
	// printf("geter %f\n", geter);

	 // C++: bool VideoCapture::set(int propId, double value)
	// cam->cap->set(CV_CAP_PROP_CONTRAST, 0.5);
	// cam->cap->set(CV_CAP_PROP_BRIGHTNESS, 0.5);
	// cam->cap->set(CV_CAP_PROP_SATURATION, 0.5);
	// cam->cap->set(CV_CAP_PROP_WHITE_BALANCE_U, 0.5);
    stop = true;
	while(cam->cap->isOpened() && stop){
		while(!cam->runing);
		cam->time = get_time();
		if(!cam->cap->read(*cam->frame_branco)) break;
		
		// imshow("Original_branco", *cam->frame_branco);
		


		cvtColor(*cam->frame_branco,*cam->gray,CV_RGB2GRAY);
		threshold(*cam->gray, *cam->threshold_img, sensitivity_branco, 255, 0);
		// imshow("trehs1", *cam->threshold_img);

        // sprintf(str, "./medidas_robocore1/threshold_img%d.bmp", ++cont);

        // imwrite(str, *cam->threshold_img);

		findContours(*cam->threshold_img, *cam->contoursB, *cam->hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
		cam->maior_areaB = 0;
		cam->contoursB_size = cam->contoursB->size();
		if(cam->contoursB_size>0){
			for (cam->i = 0; cam->i < cam->contoursB_size; cam->i++){
				cam->atual_areaB = (int)contourArea((*cam->contoursB)[cam->i]); // <--- aqui ta o erro
 				if(cam->maior_areaB < cam->atual_areaB){
 					cam->maior_areaB = cam->atual_areaB;
					cam->pos_maior_areaB = cam->i;
				}
			}
			Rect rect = boundingRect((*cam->contoursB)[cam->pos_maior_areaB]);
			
			// pthread_mutex_lock(&m->mutex);0
			cam->x = rect.x + rect.width/2- 320;
			cam->y = rect.y + rect.height/2 - 240;
			// centro_x = rect.x + rect.width/2;
			// centro_y = rect.x + rect.width/2;
			// x1 = rect.x - 320;
			// x2 = rect.x + rect.width -320;
			// // pthread_mutex_unlock(&*/m->mutex);
			// printf("\nx1 %d, x2 %d\n", x1, x2);

			printf("cam->x %d\n", cam->x);
			printf("cam->y %d\n", cam->y);

			if(ret = pthread_mutex_trylock(&axis->pmutex)){
	            // printf("if ret lock: %d\n", ret);
	            // break;
	        }
	        else{
	            // printf("else ret lock: %d\n", ret);
	            /* Lock mutex */
	            // pthread_mutex_lock(&axis->pmutex);
	            axis->x = cam->x;
	            axis->y = cam->y;
	            printf("x: %d\ty: %d\n", axis->x, axis->y);
	            /* Unlock mutex */
	            if(ret = pthread_mutex_unlock(&axis->pmutex)){
	                // printf("if ret unlock: %d\n", ret);
	                // break;
	            }
	        }

			// if(cam->x >0){
			// 	cam->d = siga;
			// 	if(rect.y > (x1 - desloc)*angulo ){
			// 	cam->d =direita;
			// 	}
			// }else {
			// 	cam->d = siga;
			// 	if(rect.y > (-x2 - desloc)*angulo ){
			// 	cam->d = esquerda;
			// 	}
			// }
			// printf("cam->d %d\n", cam->d);

			// Point pt1, pt2;
   //          pt1.x = rect.x;
   //          pt1.y = rect.y;
   //          pt2.x = rect.x + rect.width;
   //          pt2.y = rect.y + rect.height;
   //          rectangle(*cam->frame_branco, pt1, pt2, CV_RGB(255,0,0), 2);
            // printf("pt1.x %d, pt2.x %d\n", pt1.x, pt2.x);
            // printf("pt1.y %d, pt2.y %d\n", pt1.y, pt2.y);
            // imshow("threshold_img", *cam->threshold_img);
		}
		cam->time = get_time() - cam->time;
		time_frame += cam->time;
		cont_fps++;
		if(time_frame>1){
			time_fps = cont_fps;
			time_frame = 0;
			cont_fps = 0;
		}
		printf("tempo(fps): %f\ttempo(s): %f\ttempo(cont(s)): %d\n", (1/cam->time), cam->time, time_fps);
		// imshow("Original_branco dect", *cam->frame_branco);
		// int k = waitKey(10);
  //       if (k==27|| cont>200) // || cont>10
  //           break;
	}
}

void* blackTrakkingThread(void *arg) {
	camera cam = (camera)arg;
	int cont = 0;
    char str[] = "./canal_verde0.jpg";
    int centro_x, centro_y, x1, x2;
    cam->cap->set(CV_CAP_PROP_CONTRAST, 0.7);	
	cam->cap->set(CV_CAP_PROP_BRIGHTNESS, 0.5);
	cam->cap->set(CV_CAP_PROP_SATURATION, 0.5);

	while(cam->cap->isOpened()) {
		while(!cam->runing && !cam->obst);
        if(!cam->cap->read(*cam->frame_preto)) break;

        // The actual splitting.
        split(*cam->frame_preto, cam->channel); // erro aqui na segunda execucao do loop

        *cam->canal_verde = ((cam->channel)[1]) - ((cam->channel)[2]); // azul=0, verde=1, vermelho=2
        imshow("subtrao",*cam->canal_verde);

        // sprintf(str, "./Campo_teste4/frame_preto%d.jpg", ++cont);
        // imwrite(str, *cam->frame_preto);

        threshold(*cam->canal_verde, *cam->threshold_img_verde, sensitivity_preto, 255, 0); // THRESH_BINARY_INV = 1, THRESH_BINARY=0;
        imshow("threshold verde",*cam->threshold_img_verde);
        
        /// Find contours
        findContours(*cam->threshold_img_verde, *cam->contoursP, *cam->hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
        cam->maior_areaP = 0;
        cam->contoursP_size = cam->contoursP->size();
        if(cam->contoursP>0){
            for (cam->ii = 0; cam->ii < cam->contoursP_size; cam->ii++){
                cam->atual_areaP = (int)contourArea((*cam->contoursP)[cam->ii]);
                if(cam->maior_areaP < cam->atual_areaP){
                    cam->maior_areaP = cam->atual_areaP;
                    cam->pos_maior_areaP = cam->ii; 
                }
            }

            Rect rect = boundingRect((*cam->contoursP)[cam->pos_maior_areaP]);
			// pthread_mutex_lock(&m->mutex);0
			// cam->x = rect.x + rect.width/2- 320;
			// cam->y = rect.y + rect.height/2 - 240;
			centro_x = rect.x + rect.width/2- 320;
			centro_y = rect.y + rect.height/2 - 240;
			x1 = rect.x - 320;
			x2 = rect.x + rect.width -320;
			// pthread_mutex_unlock(&*/m->mutex);
			printf("\nx1 %d, x2 %d\n", x1, x2);
			printf("centro_x %d\n", centro_x);
			printf("centro_y %d\n", centro_y);

            if(centro_x > 0){
				cam->d = siga;
				if(rect.y > (x1 - desloc)*angulo ){
				cam->d =direita;
				}
			}else {
				cam->d = siga;
				if(rect.y > (-x2 - desloc)*angulo ){
				cam->d = esquerda;
				}
			}
			printf("cam->d %d\n", cam->d);

			Point pt1, pt2;
            pt1.x = rect.x;
            pt1.y = rect.y;
            pt2.x = rect.x + rect.width;
            pt2.y = rect.y + rect.height;
            rectangle(*cam->frame_preto, pt1, pt2, CV_RGB(255,0,0), 2);
        }
	    // imshow("threshold_img_verde", *cam->threshold_img_verde);
	    // imshow("canal_verde", *cam->canal_verde);
		// imshow("Original", *cam->frame_preto);
		// imwrite( "../../images/Gray_Image.jpg", gray_image );

		// int k = waitKey(10);
	 //      if (k==27 || cont>200)  //
	 //          break;
	}
}

int main(int argc, char const *argv[]){

	int fd;
    int result;
    signal(SIGINT, stopper);
    signal(SIGUSR1, stopper);
    // signal(SIGQUIT, stopper);
    signal(SIGTERM, stopper);
    
    /* Open a file for writing.
     *  - Creating the file if it doesn't exist.
     *  - Truncating it to 0 size if it already exists. (not really needed)
     *
     * Note: "O_WRONLY" mode is not sufficient when mmapping.
     */
    fd = open(FILEPATH, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU); //(mode_t)0666
    if (fd == -1)
    {
        perror("Error opening file for writing");
        exit(EXIT_FAILURE);
    }

    /* NB: ftruncate(fd, FILESIZE); is simpler */
    /* Stretch the file size to the size of the (mmapped) array of structs */
    result = lseek(fd, FILESIZE - 1, SEEK_SET);
    if (result == -1)
    {
        close(fd);
        perror("Error calling lseek() to 'stretch' the file");
        exit(EXIT_FAILURE);
    }

    /* Something needs to be written at the end of the file to
     * have the file actually have the new size.
     * Just writing an empty string at the current file position will do.
     *
     * Note:
     *  - The current position in the file is at the end of the stretched
     *    file due to the call to lseek().
     *  - An empty string is actually a single '\0' character, so a zero-byte
     *    will be written at the last byte of the file.
     */
    result = write(fd, "", 1);
    if (result != 1)
    {
        close(fd);
        perror("Error writing last byte of the file");
        exit(EXIT_FAILURE);
    }

    /* Now the file is ready to be mmapped.  */
    axis = (struct Axis_t *)mmap(NULL, FILESIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (axis == MAP_FAILED)
    {
        close(fd);
        perror("Error mmapping the file");
        exit(EXIT_FAILURE);
    }

    pthread_mutexattr_t attrmutex;
    /* Initialize attribute to mutex. */
	pthread_mutexattr_init(&attrmutex);
	pthread_mutexattr_setpshared(&attrmutex, PTHREAD_PROCESS_SHARED);

	/* Allocate memory to pmutex here. */

	/* Initialise mutex. */
	pthread_mutex_init(&axis->pmutex, &attrmutex);
	memset(axis, '\0', sizeof(Axis_t));

	/* Inicialize camera */
	camera cam = cameraCreate();

	/* Inicialize daemon process */
	pid_t pid, pid_p;

    /* Fork off the parent process */
    pid = fork();

    /* An error occurred */
    if (pid < 0)
        exit(EXIT_FAILURE);

    /* Success: Let the parent terminate */
    if (pid > 0)
        exit(EXIT_SUCCESS);

    /* On success: The child process becomes session leader */
    if (setsid() < 0)
        exit(EXIT_FAILURE);

    /* Catch, ignore and handle signals */
    //TODO: Implement a working signal handler */
    signal(SIGCHLD, SIG_IGN);
    signal(SIGHUP, SIG_IGN);

    /* Fork off for the second time*/
    pid_p = fork();

    /* An error occurred */
    if (pid_p < 0)
        exit(EXIT_FAILURE);

    /* Success: Let the parent terminate */
    if (pid_p > 0)
        exit(EXIT_SUCCESS);

    /* Set new file permissions */
    umask(0);

    /* Change the working directory to the root directory */
    /* or another appropriated directory */
    chdir("/");

    FILE *pid_process = fopen(PID_FILEPATH, "w");
    if(pid_process==NULL){
    	fprintf(stderr, "Opening file pid_process failed!\n");
    }
    fprintf(pid_process, "%d\n", getpid());
    fflush(pid_process);
    fclose(pid_process);

    /* Close open file descriptors */
    fclose(stdin);
    fclose(stderr);
    fclose(stdout);

    /* Open the log file */
    openlog ("firstdaemon", LOG_PID, LOG_DAEMON);
 
    while(sig_term){
    	if(sig_user1){
    		whiteTrakkingThread((void*)cam);
    	}
    	if(sig_user2){
    		// blackTrakkingThread((void*)cam);
    	}
    	sleep(1);
    }
	
	cam->cap->release();
	free(cam);
	free(axis);
	close(fd);
	/* Remove file? */
    unlink(FILEPATH); 
    unlink(PID_FILEPATH);

    /* Clean up. */
    pthread_mutex_unlock(&axis->pmutex);
	pthread_mutex_destroy(&axis->pmutex);
	pthread_mutexattr_destroy(&attrmutex);

	/* Close all open file descriptors */
	int x;
	for (x = sysconf(_SC_OPEN_MAX); x>=0; x--){
	    close(x);
	}
	exit(pid_p);
	exit(pid);
	exit(EXIT_SUCCESS);
	return 0;
}
