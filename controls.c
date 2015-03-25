/*
 * Control panel for IC Simulation
 *
 * OpenGarages 
 *
 * craig@theialabs.com
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#ifndef DATA_DIR
#define DATA_DIR "./data/"
#endif
#define DEFAULT_CAN_TRAFFIC DATA_DIR "sample-can.log"

#define DEFAULT_DIFFICULTY 1
// 0 = No randomization added to the packets other than location and ID
// 1 = Add NULL padding
// 2 = Randomize unused bytes
#define DEFAULT_DOOR_ID 411
#define DEFAULT_DOOR_POS 2
#define DEFAULT_SIGNAL_ID 392
#define DEFAULT_SIGNAL_POS 0
#define DEFAULT_SPEED_ID 580
#define DEFAULT_SPEED_POS 3
#define CAN_DOOR1_LOCK 1
#define CAN_DOOR2_LOCK 2 
#define CAN_DOOR3_LOCK 4
#define CAN_DOOR4_LOCK 8
#define CAN_LEFT_SIGNAL 1
#define CAN_RIGHT_SIGNAL 2
#define ON 1
#define OFF 0
#define DOOR_LOCKED 0
#define DOOR_UNLOCKED 1
#define SCREEN_WIDTH 835
#define SCREEN_HEIGHT 608
#define JOY_UNKNOWN -1
#define BUTTON_LOCK 4
#define PS3_BUTTON_LOCK 10
#define BUTTON_UNLOCK 5
#define PS3_BUTTON_UNLOCK 11
#define BUTTON_A 0
#define PS3_BUTTON_A 14
#define BUTTON_B 1
#define PS3_BUTTON_B 13
#define BUTTON_X 2
#define PS3_BUTTON_X 15 
#define BUTTON_Y 3
#define PS3_BUTTON_Y 12
#define BUTTON_START 7
#define PS3_BUTTON_START 3
#define AXIS_LEFT_V 0
#define PS3_AXIS_LEFT_V 0
#define AXIS_LEFT_H 1
#define PS3_AXIS_LEFT_H 1
#define AXIS_L2 2
#define PS3_AXIS_L2 12
#define AXIS_RIGHT_H 3
#define PS3_AXIS_RIGHT_H 3
#define AXIS_RIGHT_V 4
#define PS3_AXIS_RIGHT_V 2
#define AXIS_R2 5
#define PS3_AXIS_R2 13
#define PS3_X_ROT 4
#define PS3_Y_ROT 5
#define PS3_Z_ROT 6 // The rotations are just guessed
#define MAX_SPEED 90.0 // Limiter 260.0 is full guage speed
#define ACCEL_RATE 8.0 // 0-MAX_SPEED in seconds
#define USB_CONTROLLER 0
#define PS3_CONTROLLER 1

int gButtonY = BUTTON_Y;
int gButtonX = BUTTON_X;
int gButtonA = BUTTON_A;
int gButtonB = BUTTON_B;
int gButtonStart = BUTTON_START;
int gButtonLock = BUTTON_LOCK;
int gButtonUnlock = BUTTON_UNLOCK;
int gAxisL2 = AXIS_L2;
int gAxisR2 = AXIS_R2;
int gAxisRightH = AXIS_RIGHT_H;
int gAxisRightV = AXIS_RIGHT_V;
int gAxisLeftH = AXIS_LEFT_H;
int gAxisLeftV = AXIS_LEFT_V;
// Acelleromoter axis info
int gJoyX = JOY_UNKNOWN;
int gJoyY = JOY_UNKNOWN;
int gJoyZ = JOY_UNKNOWN;

//Analog joystick dead zone
const int JOYSTICK_DEAD_ZONE = 8000;
int gLastAccelValue = 0; // Non analog R2

int s; // socket
struct canfd_frame cf;
char *traffic_log = DEFAULT_CAN_TRAFFIC;
struct ifreq ifr;
int door_pos = DEFAULT_DOOR_POS;
int signal_pos = DEFAULT_SIGNAL_POS;
int speed_pos = DEFAULT_SPEED_POS;
int door_len = DEFAULT_DOOR_POS + 1;
int signal_len = DEFAULT_DOOR_POS + 1;
int speed_len = DEFAULT_SPEED_POS + 2;
int difficulty = DEFAULT_DIFFICULTY;

int lock_enabled = 0;
int unlock_enabled = 0;
char door_state = 0xf;
char signal_state = 0;
int throttle = 0;
float current_speed = 0;
int turning = 0;
int door_id, signal_id, speed_id;
int currentTime;
int lastAccel = 0;
int lastTurnSignal = 0;

int seed = 0;
int debug = 0;

int play_id;
int kk = 0;
char data_file[256];
SDL_GameController *gGameController = NULL;
SDL_Joystick *gJoystick = NULL;
SDL_Haptic *gHaptic = NULL;
SDL_Renderer *renderer = NULL;
SDL_Texture *base_texture = NULL;
int gControllerType = USB_CONTROLLER;

void kk_check(int);

// Adds data dir to file name
// Uses a single pointer so not to have a memory leak
// returns point to data_files or NULL if append is too large
char *get_data(char *fname) {
  if(strlen(DATA_DIR) + strlen(fname) > 255) return NULL;
  strncpy(data_file, DATA_DIR, 255);
  strncat(data_file, fname, 255-strlen(data_file));
  return data_file;
}


void send_pkt(int mtu) {
  if(write(s, &cf, mtu) != mtu) {
	perror("write");
  }
}

// Randomizes bytes in CAN packet if difficulty is hard enough
void randomize_pkt(int start, int stop) {
	if (difficulty < 2) return;
	int i = start;
	for(;i < stop;i++) {
		if(rand() % 3 < 1) cf.data[i] = rand() % 255;
	}
}

void send_lock(char door) {
	door_state |= door;
	memset(&cf, 0, sizeof(cf));
	cf.can_id = door_id;
	cf.len = door_len;
	cf.data[door_pos] = door_state;
	if (door_pos) randomize_pkt(0, door_pos);
	if (door_len != door_pos + 1) randomize_pkt(door_pos + 1, door_len);
	send_pkt(CAN_MTU);
}

void send_unlock(char door) {
	door_state &= ~door;
	memset(&cf, 0, sizeof(cf));
	cf.can_id = door_id;
	cf.len = door_len;
	cf.data[door_pos] = door_state;
	if (door_pos) randomize_pkt(0, door_pos);
	if (door_len != door_pos + 1) randomize_pkt(door_pos + 1, door_len);
	send_pkt(CAN_MTU);
}

void send_speed() {
	int kph = (current_speed / 0.6213751) * 100;
	memset(&cf, 0, sizeof(cf));
	cf.can_id = speed_id;
	cf.len = speed_len;
	cf.data[speed_pos+1] = (char)kph & 0xff;
	cf.data[speed_pos] = (char)(kph >> 8) & 0xff;
	if(kph == 0) { // IDLE
		cf.data[speed_pos] = 1;
		cf.data[speed_pos+1] = rand() % 255+100;
	}
	if (speed_pos) randomize_pkt(0, speed_pos);
	if (speed_len != speed_pos + 2) randomize_pkt(speed_pos+2, speed_len);
	send_pkt(CAN_MTU);
}

void send_turn_signal() {
	memset(&cf, 0, sizeof(cf));
	cf.can_id = signal_id;
	cf.len = signal_len;
	cf.data[signal_pos] = signal_state;
	if(signal_pos) randomize_pkt(0, signal_pos);
	if(signal_len != signal_pos + 1) randomize_pkt(signal_pos+1, signal_len);
	send_pkt(CAN_MTU);
}

// Checks throttle to see if we should accelerate or decelerate the vehicle
void checkAccel() {
	float rate = MAX_SPEED / (ACCEL_RATE * 100);
	// Updated every 10 ms
	if(currentTime > lastAccel + 10) {
		if(throttle < 0) {
			current_speed -= rate;
			if(current_speed < 1) current_speed = 0;
		} else if(throttle > 0) {
			current_speed += rate;
			if(current_speed > MAX_SPEED) { // Limiter
				current_speed = MAX_SPEED;
				if(gHaptic != NULL) {SDL_HapticRumblePlay( gHaptic, 0.5, 1000); printf("DEBUG HAPTIC\n"); }
			}
		}
		send_speed();
		lastAccel = currentTime;
	}
}

// Checks if turning and activates the turn signal
void checkTurn() {
	if(currentTime > lastTurnSignal + 500) {
		if(turning < 0) {
			signal_state ^= CAN_LEFT_SIGNAL;
		} else if(turning > 0) {
			signal_state ^= CAN_RIGHT_SIGNAL;
		} else {
			signal_state = 0;
		}
		send_turn_signal();
		lastTurnSignal = currentTime;
	}
}

// Takes R2 joystick value and converts it to throttle speed
void accelerate(int value) {
	// Check dead zones
	if(gControllerType == PS3_CONTROLLER) {
		// PS3 works different.  the value range is 0-32k
		if (value < gLastAccelValue) {
			throttle = -1;
		} else if (value > gLastAccelValue) {
			throttle = 1;
		} else {
			throttle = 0;
		}
		gLastAccelValue = value;
	} else {
		if(value < -JOYSTICK_DEAD_ZONE) {
			throttle = -1;
		} else if(value > JOYSTICK_DEAD_ZONE) {
			throttle = 1;
		} else {
			throttle = 0;
		}
	}
}

// Check LEFT_V axis to see if we are turning
void turn(int value) {
	if(value < -JOYSTICK_DEAD_ZONE) {
		turning = -1;
		kk_check(SDLK_LEFT);
	} else if(value > JOYSTICK_DEAD_ZONE) {
		turning = 1;
		kk_check(SDLK_RIGHT);
	} else {
		turning = 0;
	}
}

void ud(int value) {
	if(value < -JOYSTICK_DEAD_ZONE) {
		kk_check(SDLK_UP);
	} else if(value > JOYSTICK_DEAD_ZONE) {
		kk_check(SDLK_DOWN);
	}
}

void kkpay() {
  printf("KK\n");
}

void kk_check(int k) {
  switch(k) {
    case SDLK_RETURN:
	if(kk == 0xa) kkpay();
	kk = 0;
	break;
    case SDLK_UP:
	kk = (kk < 2) ? kk+1 : 0;
	break;
    case SDLK_DOWN:
	kk = (kk > 1 && kk < 4) ? kk+1 : 0;
	break;
    case SDLK_LEFT:
	kk = (kk == 4 || kk == 6) ? kk+1 : 0;
	break;
    case SDLK_RIGHT:
	kk = (kk == 5 || kk == 7) ? kk+1 : 0;
	break;
    case SDLK_a:
	kk = kk == 9 ? kk+1 : 0;
	break;
    case SDLK_b:
	kk = kk == 8 ? kk+1 : 0;
	break;
    default:
	kk == 0;
  }
}

// Plays background can traffic
void play_can_traffic() {
	char can2can[50];
	snprintf(can2can, 49, "%s=can0", ifr.ifr_name);
	if(execlp("canplayer", "canplayer", "-I", traffic_log, "-l", "i", can2can, NULL) == -1) printf("WARNING: Could not execute canplayer. No bg data\n");
}

void kill_child(int sig) {
	kill(play_id, SIGKILL);
}

void redraw_screen() {
  SDL_RenderCopy(renderer, base_texture, NULL, NULL);
  SDL_RenderPresent(renderer);
}

// Maps the controllers buttons
void map_joy() {
	switch(gControllerType) {
	case USB_CONTROLLER:
		gButtonA = BUTTON_A;
		gButtonB = BUTTON_B;
		gButtonX = BUTTON_X;
		gButtonY = BUTTON_Y;
		gButtonStart = BUTTON_START;
		gButtonLock = BUTTON_LOCK;
		gButtonUnlock = BUTTON_UNLOCK;
		gAxisL2 = AXIS_L2;
		gAxisR2 = AXIS_R2;
		gAxisRightH = AXIS_RIGHT_H;
		gAxisRightV = AXIS_RIGHT_V;
		gAxisLeftH = AXIS_LEFT_H;
		gAxisLeftV = AXIS_LEFT_V;
		break;
	case PS3_CONTROLLER:
		gButtonA = PS3_BUTTON_A;
		gButtonB = PS3_BUTTON_B;
		gButtonX = PS3_BUTTON_X;
		gButtonY = PS3_BUTTON_Y;
		gButtonStart = PS3_BUTTON_START;
		gButtonLock = PS3_BUTTON_LOCK;
		gButtonUnlock = PS3_BUTTON_UNLOCK;
		gAxisL2 = PS3_AXIS_L2;
		gAxisR2 = PS3_AXIS_R2;
		gAxisRightH = PS3_AXIS_RIGHT_H;
		gAxisRightV = PS3_AXIS_RIGHT_V;
		gAxisLeftH = PS3_AXIS_LEFT_H;
		gAxisLeftV = PS3_AXIS_LEFT_V;
		gJoyX = PS3_X_ROT;
		gJoyY = PS3_Y_ROT;
		gJoyZ = PS3_Z_ROT;
 		break; 
	default:
	printf("Unknown controller type for mapping\n");
  	}
}

void print_joy_info() {
	printf("Name: %s\n", SDL_JoystickNameForIndex(0));
	printf("Number of Axes: %d\n", SDL_JoystickNumAxes(gJoystick));
	printf("Number of Buttons: %d\n", SDL_JoystickNumButtons(gJoystick));
	if(SDL_JoystickNumBalls(gJoystick) > 0) printf("Number of Balls: %d\n", SDL_JoystickNumBalls(gJoystick));
        if(strncmp(SDL_JoystickNameForIndex(0), "PLAYSTATION(R)3 Controller", 25) == 0) {
		// PS3 Rumble controller via BT
		gControllerType = PS3_CONTROLLER;
	}
        if(strncmp(SDL_JoystickNameForIndex(0), "Sony PLAYSTATION(R)3 Controller", 30) == 0) {
		// PS3 directly connected
		gControllerType = PS3_CONTROLLER;
	}
	map_joy();
}

void usage(char *msg) {
  if(msg) printf("%s\n", msg);
  printf("Usage: controls [options] <can>\n");
  printf("\t-s\tseed value from IC\n");
  printf("\t-l\tdifficulty level. 0-2 (default: %d)\n", DEFAULT_DIFFICULTY);
  printf("\t-t\ttraffic file to use for bg CAN traffic\n");
  printf("\t-X\tDisable background CAN traffic.  Cheating if doing RE but needed if playing on a real CANbus\n");
  printf("\t-d\tdebug mode\n");
  exit(1);
}

int main(int argc, char *argv[]) {
  int opt;
  struct sockaddr_can addr;
  struct canfd_frame frame;
  int running = 1;
  int enable_canfd = 1;
  int play_traffic = 1;
  struct stat st;
  SDL_Event event;

  while ((opt = getopt(argc, argv, "Xdl:s:t:h?")) != -1) {
    switch(opt) {
	case 'l':
		difficulty = atoi(optarg);
		break;
	case 's':
		seed = atoi(optarg);
		break;
	case 't':
		traffic_log = optarg;
		break;
	case 'd':
		debug = 1;
		break;
	case 'X':
		play_traffic = 0;
		break;
	case 'h':
	case '?':
	default:
		usage(NULL);
		break;
    }
  }

  if (optind >= argc) usage("You must specify at least one can device");

  if(stat(traffic_log, &st) == -1) {
	char msg[256];
	snprintf(msg, 255, "CAN Traffic file not found: %s\n", traffic_log);
	usage(msg);
  }

  /* open socket */
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
       perror("socket");
       return 1;
  }

  addr.can_family = AF_CAN;

  strcpy(ifr.ifr_name, argv[optind]);
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
       perror("SIOCGIFINDEX");
       return 1;
  }
  addr.can_ifindex = ifr.ifr_ifindex;

  if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                 &enable_canfd, sizeof(enable_canfd))){
       printf("error when enabling CAN FD support\n");
       return 1;
  }

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
       perror("bind");
       return 1;
  }

  door_id = DEFAULT_DOOR_ID;
  signal_id = DEFAULT_SIGNAL_ID;
  speed_id = DEFAULT_SPEED_ID;

  if (seed) {
	srand(seed);
        door_id = (rand() % 2046) + 1;
        signal_id = (rand() % 2046) + 1;
        speed_id = (rand() % 2046) + 1;
        door_pos = rand() % 9;
        signal_pos = rand() % 9;
        speed_pos = rand() % 8;
        printf("Seed: %d\n", seed);
	door_len = door_pos + 1;
	signal_len = signal_pos + 1;
	speed_len = speed_len + 2;
  }

  if(difficulty > 0) {
	if (door_len < 8) {
		door_len += rand() % (8 - door_len);
	} else {
		door_len = 0;
	}
	if (signal_len < 8) {
		signal_len += rand() % (8 - signal_len);
	} else {
		signal_len = 0;
	}
	if (speed_len < 8) {
		speed_len += rand() % (8 - speed_len);
	} else {
		speed_len = 0;
	}
  }

  if(play_traffic) {
	signal(SIGALRM,(void (*)(int))kill_child);
	play_id = fork();
	if((int)play_id == -1) {
		printf("Error: Couldn't fork bg player\n");
		exit(-1);
	} else if (play_id != 0) {
		play_can_traffic();
	}
  }

  // GUI Setup
  SDL_Window *window = NULL;
  SDL_Surface *screenSurface = NULL;
  if(SDL_Init ( SDL_INIT_VIDEO | SDL_INIT_JOYSTICK ) < 0 ) {
        printf("SDL Could not initializes\n");
        exit(40);
  }
  if( SDL_NumJoysticks() < 1) {
	printf(" Warning: No joysticks connected\n");
  } else {
	if(SDL_IsGameController(0)) {
	  gGameController = SDL_GameControllerOpen(0);
	  if(gGameController == NULL) {
		printf(" Warning: Unable to open game controller. %s\n", SDL_GetError() );
	  } else {
		gJoystick = SDL_GameControllerGetJoystick(gGameController);
		gHaptic = SDL_HapticOpenFromJoystick(gJoystick);
		print_joy_info();
	  }
        } else {
		gJoystick = SDL_JoystickOpen(0);
		if(gJoystick == NULL) {
			printf(" Warning: Could not open joystick\n");
		} else {
			gHaptic = SDL_HapticOpenFromJoystick(gJoystick);
			if (gHaptic == NULL) printf("No Haptic support\n");
			print_joy_info();
		}
	}
  }
  window = SDL_CreateWindow("CANBus Control Panel", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
  if(window == NULL) {
        printf("Window could not be shown\n");
  }
  renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_Surface *image = IMG_Load(get_data("joypad.png"));
  base_texture = SDL_CreateTextureFromSurface(renderer, image);
  SDL_RenderCopy(renderer, base_texture, NULL, NULL);
  SDL_RenderPresent(renderer);
  int button, axis; // Used for checking dynamic joystick mappings

  while(running) {
    while( SDL_PollEvent(&event) != 0 ) {
        switch(event.type) {
            case SDL_QUIT:
                running = 0;
                break;
	    case SDL_WINDOWEVENT:
		switch(event.window.event) {
		case SDL_WINDOWEVENT_ENTER:
		case SDL_WINDOWEVENT_RESIZED:
			redraw_screen();
			break;
		}
	    case SDL_KEYDOWN:
		switch(event.key.keysym.sym) {
		    case SDLK_UP:
			throttle = 1;
			break;
		    case SDLK_LEFT:
			turning = -1;
			break;
		    case SDLK_RIGHT:
			turning = 1;
			break;
		    case SDLK_LSHIFT:
			lock_enabled = 1;
			if(unlock_enabled) send_lock(CAN_DOOR1_LOCK | CAN_DOOR2_LOCK | CAN_DOOR3_LOCK | CAN_DOOR4_LOCK);
			break;
		    case SDLK_RSHIFT:
			unlock_enabled = 1;
			if(lock_enabled) send_unlock(CAN_DOOR1_LOCK | CAN_DOOR2_LOCK | CAN_DOOR3_LOCK | CAN_DOOR4_LOCK);
			break;
		    case SDLK_a:
			if(lock_enabled) {
				send_lock(CAN_DOOR1_LOCK);
			} else if(unlock_enabled) {
				send_unlock(CAN_DOOR1_LOCK);
			}
			break;
		    case SDLK_b:
			if(lock_enabled) {
				send_lock(CAN_DOOR2_LOCK);
			} else if(unlock_enabled) {
				send_unlock(CAN_DOOR2_LOCK);
			}
			break;
		    case SDLK_x:
			if(lock_enabled) {
				send_lock(CAN_DOOR3_LOCK);
			} else if(unlock_enabled) {
				send_unlock(CAN_DOOR3_LOCK);
			}
			break;
		    case SDLK_y:
			if(lock_enabled) {
				send_lock(CAN_DOOR4_LOCK);
			} else if(unlock_enabled) {
				send_unlock(CAN_DOOR4_LOCK);
			}
			break;
		}
		kk_check(event.key.keysym.sym);
	   	break;
	    case SDL_KEYUP:
		switch(event.key.keysym.sym) {
		    case SDLK_UP:
			throttle = -1;
			break;
		    case SDLK_LEFT:
		    case SDLK_RIGHT:
			turning = 0;
			break;
		    case SDLK_LSHIFT:
			lock_enabled = 0;
			break;
		    case SDLK_RSHIFT:
			unlock_enabled = 0;
			break;
		}
		break;
	    case SDL_JOYAXISMOTION:
		axis = event.jaxis.axis;
		if(axis == gAxisLeftH) {
			ud(event.jaxis.value);
		} else if(axis == gAxisLeftV) {
			turn(event.jaxis.value);
		} else if(axis == gAxisR2) {
			accelerate(event.jaxis.value);
		} else if(axis == gAxisRightH ||
			  axis == gAxisRightV ||
			  axis == gAxisL2 ||
			  axis == gJoyX ||
			  axis == gJoyY ||
			  axis == gJoyZ) {
			// Do nothing, the axis is known just not connected
		} else {
			if (debug) printf("Unkown axis: %d\n", event.jaxis.axis);
		}
		break;
	    case SDL_JOYBUTTONDOWN:
                button = event.jbutton.button;
		if(button == gButtonLock) {
			lock_enabled = 1;
			if(unlock_enabled) send_lock(CAN_DOOR1_LOCK | CAN_DOOR2_LOCK | CAN_DOOR3_LOCK | CAN_DOOR4_LOCK);
		} else if(button == gButtonUnlock) {
			unlock_enabled = 1;
			if(lock_enabled) send_unlock(CAN_DOOR1_LOCK | CAN_DOOR2_LOCK | CAN_DOOR3_LOCK | CAN_DOOR4_LOCK);
		} else if(button == gButtonA) {
			if(lock_enabled) {
				send_lock(CAN_DOOR1_LOCK);
			} else if(unlock_enabled) {
				send_unlock(CAN_DOOR1_LOCK);
			}
			kk_check(SDLK_a);
		} else if (button == gButtonB) {
			if(lock_enabled) {
				send_lock(CAN_DOOR2_LOCK);
			} else if(unlock_enabled) {
				send_unlock(CAN_DOOR2_LOCK);
			}
			kk_check(SDLK_b);
		} else if (button == gButtonX) {
			if(lock_enabled) {
				send_lock(CAN_DOOR3_LOCK);
			} else if(unlock_enabled) {
				send_unlock(CAN_DOOR3_LOCK);
			}
			kk_check(SDLK_x);
		} else if (button == gButtonY) {
			if(lock_enabled) {
				send_lock(CAN_DOOR4_LOCK);
			} else if(unlock_enabled) {
				send_unlock(CAN_DOOR4_LOCK);
			}
			kk_check(SDLK_y);
		} else if (button == gButtonStart) {
			kk_check(SDLK_RETURN);
		} else {
			if(debug) printf("Unassigned button: %d\n", event.jbutton.button);
		}
		break;
	    case SDL_JOYBUTTONUP:
		button = event.jbutton.button;
		if(button == gButtonLock) {
			lock_enabled = 0;
		} else if(button == gButtonUnlock) {
			unlock_enabled = 0;
		} else {
			//if(debug) printf("Unassigned button: %d\n", event.jbutton.button);
		}
		break;
	    case SDL_JOYDEVICEADDED:
		// Only use the first controller
		if(event.cdevice.which == 0) {
			gJoystick = SDL_JoystickOpen(0);
			if(gJoystick) {
				gHaptic = SDL_HapticOpenFromJoystick(gJoystick);
				print_joy_info();
			}
		}
		break;
	    case SDL_JOYDEVICEREMOVED:
		if(event.cdevice.which == 0) {
			SDL_JoystickClose(gJoystick);
			gJoystick = NULL;
		}
		break;
	    case SDL_CONTROLLERDEVICEADDED:
		// Only use the first controller
		if(gGameController == NULL) {
			gGameController = SDL_GameControllerOpen(0);
			gJoystick = SDL_GameControllerGetJoystick(gGameController);
			gHaptic = SDL_HapticOpenFromJoystick(gJoystick);
			print_joy_info();
		}
		break;
	    case SDL_CONTROLLERDEVICEREMOVED:
		if(event.cdevice.which == 0) {
			SDL_GameControllerClose(gGameController);
			gGameController = NULL;
		}
		break;
        }
    }
    currentTime = SDL_GetTicks();
    checkAccel();
    checkTurn();
    SDL_Delay(5);
  }

  kill_child(SIGKILL);
  close(s);
  SDL_DestroyTexture(base_texture);
  SDL_FreeSurface(image);
  SDL_GameControllerClose(gGameController);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

}
